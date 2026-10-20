#pragma once

// Library components: parametric motors and gears that drop into the model as
// ready-made bodies, with motions where the part moves on its own (motors) or
// with its mating parts (gear trains).

#include "core/Geometry.hpp"
#include "core/Motion.hpp"

#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sicdm {

class Scene;
struct Body;

// Body kinds built by the library (see buildLibraryPart()). Parameters, all mm / deg:
//   gear:          module teeth width bore helix style(0 spur, 1 helical, 2 herringbone) phase
//   ring_gear:     module teeth width rim phase                (internal teeth)
//   bevel_gear:    module teeth face cone bore phase           (cone = pitch cone angle)
//   rack:          module teeth width height                   (teeth along X, pointing +Y, pitch line y = 0)
//   worm:          module starts length phase                  (axis along X)
//   dc_motor, stepper_motor, servo_motor: none (housing, shaft along +Z through the origin)
//   ev_motor:      none (foot-mounted traction motor, shaft along +X at z = 170, feet at z = 0)
//   ev_flange_motor: none (flange-mount traction motor, shaft along +X at z = 120)
//   motor_rotor:   style (0 DC, 1 stepper, 3 / 4 traction): style shaft_length shafts rear_length shaft_dia fitting
//                  style 2 (servo): style horn            (see partParams())
//   tyre: diameter width rim_dia tread     alloy_rim / wheel_hub, caster_fork, roller_hub, wheel_rollers: wheel parts
//   sprocket: pitch teeth width bore       pulley: type diameter width bore       chain: pitch links
//   chain_loop: pitch r1 r2 centre z phase (pins on the loop around two pitch circles, shifted by phase mm)
//   belt: type r1 r2 centre width thickness z phase                                 o_ring: inner_dia section
//   phalanx: length width thickness yaw tip pad   robot_palm: width length thickness hand(0 right, 1 left) pad
//   (pad 1 = the grip pad instead of the shell)   robot_forearm: width length thickness clevis puck   (robot hand links: joint axis along X through the origin)
//   arm_link: length radius (shoulder ball at the origin, elbow hub at y = length)   robot_shoulder: height width
//   Humanoid body (faces +Y; origin on the joint the part turns about):
//   robot_head: width depth height part(0 shell, 1 visor)   robot_neck: length radius   robot_chest: width depth height
//   robot_heart: radius part(0 housing, 1 core, 2 pump impeller)   robot_gut: width depth height part(0 plates, 1 core)
//   robot_midriff: width depth height   robot_pelvis: width depth height hip_spacing   robot_thigh / robot_shin: length width depth
//   robot_foot: length width ankle part(0 foot, 1 toe on its joint, 2 foot sole, 3 toe sole)
bool isLibraryKind(const std::string& kind);
// Chains and belts that can travel along their loop (chain_loop, belt).
bool isLoopKind(const std::string& kind);
// Mesh of a chain / belt body moved `travel` mm along its loop (scale applied).
MeshData loopMeshAt(const Body& body, float travel);
MeshData buildLibraryPart(const std::string& kind, const std::vector<float>& params);

// Involute outline of a gear in the XY plane (counter-clockwise, star-shaped
// around the origin), tooth 0 centred on `phaseRad`. Exposed for tests.
std::vector<Vec2> gearOutline(float module, int teeth, float phaseRad, float addendum = 1.0f, float dedendum = 1.25f);

// One editable parameter of a library item.
struct LibraryParam {
    const char* key;   // command keyword, e.g. "teeth"
    const char* label; // UI label
    float value;       // default
    float min, max;
    const char* unit;  // "mm", "deg", "rpm", "" ...
    bool integer = false;
    const char* choices = nullptr; // "None|Single|Dual": value is the index (shown as a list)
};

// Editable parameters of a placed library body (index = position in Body::params),
// e.g. a motor rotor's shafts and lengths or a gear's teeth. Empty for other kinds.
std::vector<LibraryParam> partParams(const std::string& kind, const std::vector<float>& params);
// The full parameter list of a body, missing trailing values filled with defaults.
std::vector<float> withDefaults(const std::string& kind, std::vector<float> params);

struct LibraryItem {
    const char* key;      // "dc_motor", "spur_gear", ...
    const char* aliases;  // space separated, upper case: "DCMOTOR DC"
    const char* name;     // "DC Motor"
    const char* category; // "Motors", "Gears", "Linear gears", "Gear trains"
    const char* summary;  // one line for the library panel
    std::vector<LibraryParam> params;
};

const std::vector<LibraryItem>& libraryItems();
const LibraryItem* findLibraryItem(std::string_view keyOrAlias);

// A placed component: its bodies (tagged with the component name) and motions.
struct LibraryComponent {
    std::string name;
    std::vector<Body> bodies;
    std::vector<Motion> motions;
};

// Builds `item` with `values` (missing keys take the defaults) at `at`.
// Body and motion names start with `name`, e.g. "DC Motor 1", "DC Motor 1 Rotor".
LibraryComponent makeLibraryComponent(const LibraryItem& item, const std::map<std::string, float>& values,
                                      const std::string& name, const Vec3& at);

} // namespace sicdm

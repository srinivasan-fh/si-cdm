#pragma once

// Physical environment for the Simulation: rigid bodies moved by forces instead of
// prescribed motions.
//
//   Motion & forces    uniform gravity or Newton's gravitation between all masses,
//                      Newton's second law (F = dp/dt), momentum-conserving collisions,
//                      position / velocity / acceleration of every body.
//   Energy & work      kinetic, gravitational, electric and thermal energy, work of the
//                      applied forces, heat lost to the air: their sum is conserved.
//   Fields & waves     charges (Coulomb), uniform electric and magnetic fields (Lorentz
//                      force), wave sources (sound, light, ripples) with wavelength and
//                      Doppler-shifted frequency at every other body.
//   Thermodynamics     body temperatures: collisions and friction turn kinetic energy
//                      into heat, touching bodies conduct it, the air cools them.
//   Modern physics     special relativity (momentum gamma m v, nothing reaches c,
//                      proper time, length contraction; c can be lowered to see it) and
//                      quantum readouts (de Broglie wavelength, uncertainty limit).
//
// The run is deterministic: it is integrated once from t = 0 in SI units (fixed
// 0.5 ms steps) and cached, so the timeline scrubs, loops and plays back exactly.
// Scene lengths stay in mm.

#include "core/Math.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

class Scene;
struct Body;

enum class GravityMode { Off, Uniform, Newton, Both };
enum class WaveKind { Off, Sound, Light, Custom };

const char* gravityModeKey(GravityMode m) noexcept;
const char* waveKindKey(WaveKind k) noexcept;
std::optional<GravityMode> gravityModeFromKey(const std::string& key) noexcept;
std::optional<WaveKind> waveKindFromKey(const std::string& key) noexcept;

// Physical constants (SI, CODATA 2018).
inline constexpr double kGravitationalConstant = 6.67430e-11; // m^3 / (kg s^2)
inline constexpr double kCoulombConstant = 8.9875517923e9;    // N m^2 / C^2
inline constexpr double kLightSpeed = 299792458.0;            // m/s
inline constexpr double kPlanck = 6.62607015e-34;             // J s
inline constexpr double kSoundSpeed = 343.0;                  // m/s in air at 20 C
inline constexpr double kStandardGravity = 9.80665;           // m/s^2

// The world all physics bodies live in (saved with the project).
struct PhysicsSettings {
    bool enabled = false;
    GravityMode gravity = GravityMode::Uniform;
    Vec3 g{0.0f, 0.0f, -static_cast<float>(kStandardGravity)}; // m/s^2 (uniform gravity)
    double bigG = kGravitationalConstant;                      // Newton's gravitation
    bool floor = true;          // solid ground at z = 0
    float friction = 0.4f;      // floor friction coefficient
    bool air = false;           // quadratic air drag (sea-level air, sphere drag coefficient)
    Vec3 wind{};                // m/s: the air's velocity, felt by robots (drones, rovers) in the robot engines
    Vec3 electricField{};       // V/m, uniform
    Vec3 magneticField{};       // T, uniform
    float ambient = 293.15f;    // K, air and floor temperature
    float convection = 10.0f;   // W/(m^2 K), heat lost to the air
    float conductance = 2000.0f;// W/(m^2 K), heat flow between touching bodies
    bool relativity = false;    // special relativity (lower c to see its effects)
    double lightSpeed = kLightSpeed; // m/s
    float duration = 10.0f;     // s simulated
};

// A body's physical properties. Lengths and speeds in mm and mm/s like the scene.
struct PhysicsProps {
    bool enabled = false;       // takes part in the physics
    bool fixed = false;         // does not move (an anchor, a charged plate, a planet), but
                                // still attracts, repels and collides; follows its motions
    float mass = 0.0f;          // kg; 0 = volume x density
    float density = 1000.0f;    // kg/m^3
    Vec3 velocity{};            // mm/s at t = 0
    Vec3 spin{};                // deg/s about world X, Y, Z (kept: no torques)
    double charge = 0.0;        // C
    float temperature = 293.15f;// K at t = 0
    float specificHeat = 900.0f;// J/(kg K)
    float restitution = 0.6f;   // bounce: 1 elastic, 0 sticks
    Vec3 force{};               // N, applied all the time
    WaveKind wave = WaveKind::Off;
    float waveFrequency = 2.0f; // Hz
    float waveSpeed = 1000.0f;  // mm/s (Custom waves: water ripples, slow motion)
};

// Energy of the whole system (J).
struct PhysicsEnergy {
    double kinetic = 0, gravity = 0, electric = 0, thermal = 0;
    double toAir = 0;         // heat carried away by the air (drag and cooling)
    double externalWork = 0;  // work done on the system by applied forces
    // Conserved: kinetic + potential + thermal + heat to the air - external work.
    double balance() const noexcept { return kinetic + gravity + electric + thermal + toAir - externalWork; }
};

// One body at one instant (mm, mm/s, mm/s^2 like the scene; force in N).
struct PhysicsBodyState {
    Vec3 position;     // centre of mass
    Vec3 velocity, acceleration;
    Mat4 rotation = Mat4::identity();
    Vec3 force;        // net force (N), contacts excluded
    float temperature = 0.0f;
    double properTime = 0.0;
    bool touching = false; // on the floor or against another body
};

struct PhysicsFrame {
    float t = 0.0f;
    std::vector<PhysicsBodyState> bodies;
    PhysicsEnergy energy;
    Vec3 momentum; // kg m/s, all moving bodies
};

struct PhysicsBodyInfo {
    std::size_t body = 0;   // scene index
    std::string name;
    double mass = 0;        // kg
    double radius = 0;      // m, collision sphere
    double area = 0;        // m^2, surface
    Vec3 restCentre;        // mm, centre of mass in the model
    PhysicsProps props;
};

// A complete run over [0, settings.duration].
struct PhysicsRun {
    PhysicsSettings settings;
    std::vector<PhysicsBodyInfo> bodies;
    std::vector<PhysicsFrame> frames; // every frameStep
    float frameStep = 0.005f;
    int collisions = 0;

    // Interpolated state at time t (held after the end).
    PhysicsFrame at(float t) const;
    // Index into `bodies` of a scene body, or -1.
    int indexOf(std::size_t sceneBody) const noexcept;
};

// The run for the scene's current physics (computed once, then cached); null when
// physics is off or no body takes part.
std::shared_ptr<const PhysicsRun> physicsRun(const Scene& scene);

// Per-body transforms at time t: the motions, with physics bodies placed by the run.
std::vector<Mat4> simulationTransforms(const Scene& scene, float t);

// Mass (kg), volume (m^3) and centre of mass (mm) of a body's mesh.
struct MassProperties {
    double volume = 0, mass = 0, area = 0;
    Vec3 centre;
};
MassProperties massProperties(const Body& body);

// Human-readable state at time t: the world, conservation totals and every body
// (or only `body`): kinematics, forces, momentum, energy, heat, relativity,
// quantum and wave readouts.
std::vector<std::string> physicsReport(const Scene& scene, float t, std::optional<std::size_t> body = std::nullopt);

// Ready-made experiments (PHYSICS DEMO name): replaces the scene.
struct PhysicsDemo {
    const char* key;
    const char* title;
    const char* summary;
};
const std::vector<PhysicsDemo>& physicsDemos();
bool loadPhysicsDemo(Scene& scene, const std::string& key);

} // namespace sicdm

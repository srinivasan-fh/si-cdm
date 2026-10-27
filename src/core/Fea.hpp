#pragma once

// Finite element analysis of one body: linear static stress (forces, pressures, bearing
// loads, its own weight, thermal expansion), natural frequencies, steady heat conduction.
//
// The body is filled with cubes (8-node hexahedra on a regular grid), fixed where told, and
// loaded with forces spread over chosen faces; the stiffness equations are solved by
// conjugate gradients (element by element, no matrix kept), and the stresses found at each
// cube's centre. Units: mm, N, MPa. Small displacements, linear elastic, isotropic.
//
// Selections pick nodes on the surface: FACE x,y,z (the flat face nearest the point: surface
// nodes on its plane), NEAR x,y,z r (surface nodes within r of the point).

#include "core/Geometry.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

struct FeaMaterial {
    std::string name;
    double E = 200000;     // Young's modulus, MPa
    double nu = 0.3;       // Poisson's ratio
    double yield = 250;    // yield strength, MPa
    double density = 7.85; // g/cm3
    double conductivity = 50; // W/(m K)
    double expansion = 12;    // thermal expansion, 1e-6 / K
};
// Steel, stainless, aluminium, titanium, brass, copper, ABS, PLA, PETG, nylon.
std::optional<FeaMaterial> feaMaterial(const std::string& name);
const std::vector<FeaMaterial>& feaMaterials();

struct FeaSelection {
    enum class Kind { Face, Near } kind = Kind::Face;
    Vec3 point{};
    float radius = 0;
};

struct FeaLoad {
    FeaSelection where;
    Vec3 force{}; // N, the total, spread evenly over the nodes picked
};

struct FeaPressure {
    FeaSelection where;
    double mpa = 0; // pushing on the surface (into the body), N/mm2
};

struct FeaBearing { // a pin pushing on a hole: spread over the side facing the force
    Vec3 center{};    // on the hole's axis
    float radius = 0; // takes surface nodes this close to the centre
    Vec3 force{};     // N
};

struct FeaTemperature {
    FeaSelection where;
    double celsius = 20;
};

struct FeaHeat {
    FeaSelection where;
    double watts = 0; // flowing in (negative: out)
};

struct FeaSetup {
    FeaMaterial material;
    std::vector<FeaSelection> fixed;
    std::vector<FeaLoad> loads;
    std::vector<FeaPressure> pressures;
    std::vector<FeaBearing> bearings;
    Vec3 gravity{};          // in g (9.81 m/s2): the body's own weight
    // Heat: temperatures held on faces, heat flowing in; the body conducts it (steady state).
    std::vector<FeaTemperature> temperatures;
    std::vector<FeaHeat> heats;
    double reference = 20;   // the temperature it is free of stress at
    bool thermalStress = false; // the static analysis loads it with its expansion too
    int modes = 0;           // > 0: its lowest natural frequencies (with the fixings)
    float cell = 0;         // cube size, mm (0: about 40 across the body's longest side)
    int maxCells = 120000;  // the cube size grows until the body takes at most this many
    int maxIterations = 20000;
    double tolerance = 1e-7; // relative residual
};

struct FeaResult {
    float cell = 0;
    int cells = 0, nodes = 0, fixedNodes = 0, loadedNodes = 0, iterations = 0;
    bool converged = false;
    double maxStress = 0;      // von Mises, MPa
    Vec3 maxStressAt{};
    double maxDisplacement = 0; // mm
    Vec3 maxDisplacementAt{};
    double safetyFactor = 0;   // yield / max stress
    std::vector<float> surfaceStress;  // per vertex of the world mesh given, MPa
    std::vector<Vec3> surfaceDisplacement; // per vertex, mm
    double mass = 0;                   // kg (the cubes')
    // Natural frequencies (Hz, lowest first) and each mode's shape at the surface (largest 1).
    std::vector<double> frequencies;
    std::vector<std::vector<Vec3>> modeShapes;
    // Heat: the temperatures (C) and where the extremes are.
    double minTemperature = 0, maxTemperature = 0;
    Vec3 minTemperatureAt{}, maxTemperatureAt{};
    std::vector<float> surfaceTemperature;
};

// Runs the analysis on a closed world mesh. False (why) when the setup leaves the body free
// to move, picks nothing, or the body can't be filled. With modes set it finds natural
// frequencies (loads optional); with temperatures and no loads, the heat flow only.
bool runFea(const MeshData& world, const FeaSetup& setup, FeaResult& out, std::string* why = nullptr);

} // namespace sicdm

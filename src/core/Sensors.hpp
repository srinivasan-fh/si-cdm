#pragma once

// Simulated sensors on robot parts (RIG SENSOR), read from the simulated scene at any time:
//
//   IMU           specific force (the acceleration minus gravity, m/s^2) and angular rate
//                 (rad/s) in the sensor's frame
//   GPS           position (m, local: x east, y north, z up) and velocity, and latitude,
//                 longitude and altitude from where the scene's origin is on the Earth
//   force-torque  the force (N) and torque (N m) the joint under the part carries, at the
//                 sensor, in its frame (needs the robot simulated: RIG SIMULATE ON)
//   camera        RGB, depth (m) and segmentation (each part's id, and its class: sky,
//                 ground, robot, other) by casting a ray through every pixel of a pinhole
//                 camera ("robot": the robot's parts; for the scene as a robot, its jointed ones)
//   LiDAR         rings of beams around the sensor's up axis: each a range (m) and a point in
//                 the sensor's frame
//
// Noise models: white Gaussian noise on every reading, and for IMUs a bias that wanders as a
// random walk; typical MEMS / survey figures, scaled by the sensor's NOISE and BIAS factors
// (0 makes it ideal). Noise is drawn from the sensor's seed and the sample's time, so the same
// reading comes back every time it is asked for.
//
// Frames: a sensor's x axis is where it looks, z its up; y completes them (left).

#include "core/Rig.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace sicdm {

class Scene;

struct SensorReading {
    SensorKind kind = SensorKind::Imu;
    double t = 0;
    std::vector<std::string> labels; // IMU, GPS, force-torque: one per value
    std::vector<double> values;
    // Camera: width x height, rows top to bottom.
    int width = 0, height = 0;
    std::vector<std::uint8_t> rgb;    // 3 per pixel
    std::vector<float> depth;         // m along the view axis, 0 = nothing within range
    std::vector<int> instance;        // scene body index, -1 sky, -2 the floor
    std::vector<std::uint8_t> semantic; // 0 sky, 1 ground (floor, terrain), 2 robot, 3 other
    // LiDAR: points in the sensor's frame (m), and their ranges.
    std::vector<std::array<float, 3>> points;
    std::vector<float> ranges;
};

// Typical noise of a sensor kind at NOISE 1: white noise sigma, and bias random walk sigma
// (per root second), per value.
struct SensorNoise {
    double white = 0, walk = 0;
};
SensorNoise typicalNoise(SensorKind kind, int value);

// Reads a sensor at time t (s) of the simulation.
bool readSensor(const Scene& scene, const RigSettings& robot, const SensorSpec& sensor, float t, SensorReading& out, std::string* error = nullptr);

// Writes what a camera or LiDAR sees at time t: a camera to `stem`_rgb.png, _depth.png
// (16-bit, mm) and _segmentation.png; a LiDAR to a .csv (x,y,z,range per point) or .pcd.
// Returns the files written.
std::vector<std::string> captureSensor(const Scene& scene, const RigSettings& robot, const SensorSpec& sensor, float t, const std::string& path,
                                       std::string* error = nullptr);

// Writes an IMU, GPS or force-torque sensor's readings over [from, to] at its rate as CSV.
bool logSensor(const Scene& scene, const RigSettings& robot, const SensorSpec& sensor, float from, float to, const std::string& path,
               std::string* error = nullptr);

// One line describing a reading (scalar sensors) or a summary (camera, LiDAR).
std::string describeReading(const SensorReading& r);

} // namespace sicdm

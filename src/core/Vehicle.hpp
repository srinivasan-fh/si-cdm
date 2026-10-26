#pragma once

// Vehicle models and controllers for the robot engine: drones and rovers.
//
//   Rotors   each ROTOR link pushes along its axis with T = kT w^2 and is held back by
//            Q = kQ w^2 about it (so the frame turns the other way: that is how a
//            multirotor yaws). kT and kQ come from the rotor's diameter D with typical
//            propeller coefficients: kT = 0.1 rho D^4 / (4 pi^2), kQ = 0.008 rho D^5 / (4 pi^2).
//   Air      every other link feels quadratic drag, 1/2 rho Cd A |v - wind| (v - wind),
//            A its mean projected area, Cd 1.
//   Wheels   grip comes from the ground contact (stick-slip friction); a rover is driven
//            by its wheels' speeds.
//
// A flight controller flies a multirotor from velocity or position setpoints (and a yaw
// rate), as an autopilot's offboard mode does: the velocity error sets the thrust vector,
// that sets the attitude, the attitude error sets the body torques, and a mixer turns
// thrust and torques into each rotor's speed. A drive controller runs a rover's wheels as a
// differential drive (forward speed and turn rate).

#include "core/Multibody.hpp"

#include <vector>

namespace sicdm {

struct RotorModel {
    std::size_t joint = 0;
    int link = 0;
    double kT = 0, kQ = 0;
    int spin = 1;          // +1 turns counter-clockwise seen from above, -1 clockwise
    V3d axisUp{0, 0, 1};   // the rotor's axis, pointing up at rest
    V3d offset{};          // from the robot's centre of mass at rest (m)
    double jointSign = 1;  // the joint's axis . axisUp
    double diameter = 0;
};

struct WheelModel {
    std::size_t joint = 0;
    int link = 0;
    double radius = 0;
    V3d axle{0, 1, 0};     // its axis at rest
    double lateral = 0;    // offset from the centre of mass along the axle (m): left > 0
    double jointSign = 1;  // the joint's axis . axle
};

struct VehicleModel {
    std::vector<RotorModel> rotors;
    std::vector<WheelModel> wheels;
    std::vector<double> dragArea; // per link (m^2), 0 for rotors
    double airDensity = 1.225;
    V3d wind{0, 0, 0};
    V3d forward{1, 0, 0};         // a rover's forward direction at rest
    double hoverSpeed = 0;        // rad/s each rotor turns at to hover (multirotors)
};

VehicleModel vehicleModel(const RobotRig& rig, const V3d& wind);
// Sizes the rotor motors (where the rig doesn't say): enough torque for four times the
// hover drag, and to spin up to hover speed in 0.2 s; top speed 2.5 x hover.
void sizeRotorMotors(RobotRig& rig, const VehicleModel& model);

// The engine step a vehicle needs: a rotor turns at most 0.1 rad per step at hover speed
// (spinning faster per step, the explicitly integrated gyroscopic terms of the rotors pump
// energy into the frame's tilt and it wobbles).
double vehicleStep(const VehicleModel& model, double dt);

// Adds the rotors' thrust and drag torque and the air's drag to `mb` before every step.
void addVehicleForces(Multibody& mb, const VehicleModel& model);

// Flies a multirotor: set the setpoint, then it steers the rotors before every step.
class FlightController {
public:
    explicit FlightController(const RobotRig& rig, const VehicleModel& model);
    void hover();                                       // hold where it is now
    void velocity(const V3d& v, double yawRate = 0);    // m/s (world), rad/s
    void moveTo(const V3d& p, double yawRate = 0);      // m (world)
    void update(Multibody& mb);                         // call before each step
    const std::vector<std::size_t>& joints() const { return m_joints; }

private:
    enum class Mode { Hover, Velocity, Position } m_mode = Mode::Hover;
    V3d m_target{}, m_hold{};
    bool m_holdSet = false;
    double m_yawRate = 0, m_yaw = 0;
    bool m_yawSet = false;
    double m_mass = 0;
    std::array<double, 9> m_inertia{}; // about the centre of mass, base frame at rest
    VehicleModel m_model;
    std::vector<std::size_t> m_joints;
    double m_last = -1;
    V3d m_integral{0, 0, 0};
};

// Drives a wheeled rover: forward speed (m/s) and turn rate (rad/s, counter-clockwise).
class DriveController {
public:
    explicit DriveController(const VehicleModel& model) : m_model(model) {
        for (const WheelModel& w : model.wheels) m_joints.push_back(w.joint);
    }
    void drive(double speed, double turnRate) {
        if (turnRate != m_turn) m_integral = 0;
        m_speed = speed, m_turn = turnRate;
    }
    void update(Multibody& mb);
    const std::vector<std::size_t>& joints() const { return m_joints; }

private:
    VehicleModel m_model;
    double m_speed = 0, m_turn = 0, m_integral = 0;
    std::vector<std::size_t> m_joints;
};

// Whole-robot centre of mass velocity (m/s).
V3d comVelocity(const Multibody& mb);

} // namespace sicdm

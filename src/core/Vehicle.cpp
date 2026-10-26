#include "core/Vehicle.hpp"

#include <algorithm>
#include <cmath>

namespace sicdm {

namespace {

constexpr double kPiV = 3.14159265358979323846;
constexpr double kG = 9.80665;

V3d add(const V3d& a, const V3d& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V3d sub(const V3d& a, const V3d& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3d mul(const V3d& a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
double dot(const V3d& a, const V3d& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
V3d cross(const V3d& a, const V3d& b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
double norm(const V3d& a) { return std::sqrt(dot(a, a)); }
V3d unit(const V3d& a) {
    const double n = norm(a);
    return n > 1e-12 ? mul(a, 1.0 / n) : V3d{0, 0, 1};
}
// R v and R^T v for a row-major 3 x 3.
V3d rot(const std::array<double, 9>& R, const V3d& v) {
    return {R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2], R[6] * v[0] + R[7] * v[1] + R[8] * v[2]};
}
V3d rotT(const std::array<double, 9>& R, const V3d& v) {
    return {R[0] * v[0] + R[3] * v[1] + R[6] * v[2], R[1] * v[0] + R[4] * v[1] + R[7] * v[2], R[2] * v[0] + R[5] * v[1] + R[8] * v[2]};
}

// Solves the 4 x 4 system M x = b (Gaussian elimination); false if singular.
bool solve4(double M[4][5], double x[4]) {
    for (int c = 0; c < 4; ++c) {
        int p = c;
        for (int r = c + 1; r < 4; ++r)
            if (std::fabs(M[r][c]) > std::fabs(M[p][c])) p = r;
        if (std::fabs(M[p][c]) < 1e-18) return false;
        if (p != c)
            for (int k = 0; k < 5; ++k) std::swap(M[p][k], M[c][k]);
        for (int r = 0; r < 4; ++r) {
            if (r == c) continue;
            const double f = M[r][c] / M[c][c];
            for (int k = c; k < 5; ++k) M[r][k] -= f * M[c][k];
        }
    }
    for (int i = 0; i < 4; ++i) x[i] = M[i][4] / M[i][i];
    return true;
}

V3d restCom(const RobotRig& rig) {
    V3d c{0, 0, 0};
    double m = 0;
    for (const RigLink& l : rig.links) {
        c = add(c, mul(V3d{l.com[0], l.com[1], l.com[2]}, l.mass));
        m += l.mass;
    }
    return m > 0 ? mul(c, 1.0 / m) : c;
}

} // namespace

VehicleModel vehicleModel(const RobotRig& rig, const V3d& wind) {
    VehicleModel vm;
    vm.wind = wind;
    vm.dragArea.assign(rig.links.size(), 0.0);
    const V3d com = restCom(rig);
    for (std::size_t l = 0; l < rig.links.size(); ++l) {
        const RigLink& L = rig.links[l];
        const double sx = L.boxMax[0] - L.boxMin[0], sy = L.boxMax[1] - L.boxMin[1], sz = L.boxMax[2] - L.boxMin[2];
        const V3d centre{(L.boxMax[0] + L.boxMin[0]) * 0.5, (L.boxMax[1] + L.boxMin[1]) * 0.5, (L.boxMax[2] + L.boxMin[2]) * 0.5};
        if (L.parentJoint < 0 || (L.tag != RigTag::Rotor && L.tag != RigTag::Wheel)) {
            if (!L.bodies.empty()) vm.dragArea[l] = (sx * sy + sy * sz + sx * sz) / 3.0;
            continue;
        }
        const std::size_t j = static_cast<std::size_t>(L.parentJoint);
        const RigJoint& J = rig.joints[j];
        const V3d axis{J.axis[0], J.axis[1], J.axis[2]};
        // The disc's diameter: its largest extent across the axis.
        const double across = std::max({std::fabs(axis[0]) < 0.9 ? sx : 0.0, std::fabs(axis[1]) < 0.9 ? sy : 0.0, std::fabs(axis[2]) < 0.9 ? sz : 0.0});
        if (L.tag == RigTag::Rotor) {
            RotorModel r;
            r.joint = j;
            r.link = static_cast<int>(l);
            r.axisUp = axis[2] >= 0 ? axis : mul(axis, -1.0);
            r.jointSign = dot(axis, r.axisUp) >= 0 ? 1.0 : -1.0;
            r.offset = sub(centre, com);
            r.diameter = across;
            const double D = std::max(across, 0.01), rho = vm.airDensity;
            r.kT = 0.1 * rho * std::pow(D, 4) / (4 * kPiV * kPiV);
            r.kQ = 0.008 * rho * std::pow(D, 5) / (4 * kPiV * kPiV);
            vm.rotors.push_back(r);
        } else {
            WheelModel w;
            w.joint = j;
            w.link = static_cast<int>(l);
            w.radius = std::max(across * 0.5, 0.005);
            w.axle = unit(axis);
            w.jointSign = 1.0;
            vm.wheels.push_back(w);
        }
    }
    // Rotors alternate their spin around the centre (so their drag torques cancel at hover).
    std::vector<std::size_t> order(vm.rotors.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        return std::atan2(vm.rotors[a].offset[1], vm.rotors[a].offset[0]) < std::atan2(vm.rotors[b].offset[1], vm.rotors[b].offset[0]);
    });
    for (std::size_t k = 0; k < order.size(); ++k) vm.rotors[order[k]].spin = k % 2 == 0 ? 1 : -1;
    double kTsum = 0;
    for (const RotorModel& r : vm.rotors) kTsum += r.kT;
    if (kTsum > 0) vm.hoverSpeed = std::sqrt(rig.totalMass() * kG / kTsum);
    // A rover's axle sets its sides; forward is across the axles, level.
    if (!vm.wheels.empty()) {
        V3d axle = vm.wheels[0].axle;
        if (axle[1] < 0 || (std::fabs(axle[1]) < 1e-9 && axle[0] < 0)) axle = mul(axle, -1.0);
        vm.forward = unit(cross(axle, V3d{0, 0, 1}));
        for (WheelModel& w : vm.wheels) {
            w.jointSign = dot(w.axle, axle) >= 0 ? 1.0 : -1.0;
            w.axle = axle;
            const RigLink& L = rig.links[static_cast<std::size_t>(w.link)];
            w.lateral = dot(sub(V3d{L.com[0], L.com[1], L.com[2]}, com), axle);
        }
    }
    return vm;
}

void sizeRotorMotors(RobotRig& rig, const VehicleModel& model) {
    for (const RotorModel& r : model.rotors) {
        RigJoint& J = rig.joints[r.joint];
        if (J.effort > 0) continue;
        const RigLink& L = rig.links[static_cast<std::size_t>(r.link)];
        const V3d a = r.axisUp;
        double spin = 0; // the rotor's inertia about its axis
        for (int i = 0; i < 3; ++i)
            for (int k = 0; k < 3; ++k) spin += a[static_cast<std::size_t>(i)] * L.inertia[static_cast<std::size_t>(i * 3 + k)] * a[static_cast<std::size_t>(k)];
        const double w = std::max(model.hoverSpeed, 1.0);
        J.effort = std::max({0.05, 4.0 * r.kQ * w * w, spin * w / 0.2});
        J.velocity = 2.5 * w;
    }
}

void addVehicleForces(Multibody& mb, const VehicleModel& model) {
    mb.addForceHook([model](Multibody& m) {
        const std::vector<double> qd = m.qd();
        const RobotRig& rig = m.rig();
        for (const RotorModel& r : model.rotors) {
            const double w = qd[r.joint];
            const LinkPose p = m.linkPose(r.link);
            const V3d up = rot(p.R, r.axisUp); // the rotor's spin leaves its axis where it is
            const V3d at = m.linkCom(r.link);
            const V3d thrust = mul(up, r.kT * w * w);
            // Drag torque against the spin (about the joint's axis, whose rate w is).
            const V3d jointAxis = mul(up, r.jointSign);
            const V3d drag = mul(jointAxis, -r.kQ * w * std::fabs(w));
            m.applyWrench(r.link, thrust, drag, at);
        }
        // Air drag on everything else.
        for (std::size_t l = 0; l < rig.links.size(); ++l) {
            if (l >= model.dragArea.size() || model.dragArea[l] <= 0) continue;
            const V3d v = sub(m.linkVelocity(static_cast<int>(l)), model.wind);
            const double s = norm(v);
            if (s < 1e-9) continue;
            m.applyWrench(static_cast<int>(l), mul(v, -0.5 * model.airDensity * 1.0 * model.dragArea[l] * s), {0, 0, 0}, m.linkCom(static_cast<int>(l)));
        }
    });
}

V3d comVelocity(const Multibody& mb) {
    const RobotRig& rig = mb.rig();
    V3d v{0, 0, 0};
    double m = 0;
    for (std::size_t l = 0; l < rig.links.size(); ++l) {
        v = add(v, mul(mb.linkVelocity(static_cast<int>(l)), rig.links[l].mass));
        m += rig.links[l].mass;
    }
    return m > 0 ? mul(v, 1.0 / m) : v;
}

FlightController::FlightController(const RobotRig& rig, const VehicleModel& model) : m_model(model) {
    m_mass = rig.totalMass();
    const V3d com = restCom(rig);
    // Composite inertia about the centre of mass (rest, world axes = the base's at rest).
    std::array<double, 9> I{};
    for (const RigLink& L : rig.links) {
        const V3d d{L.com[0] - com[0], L.com[1] - com[1], L.com[2] - com[2]};
        const double dd = dot(d, d);
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b)
                I[static_cast<std::size_t>(a * 3 + b)] += L.inertia[static_cast<std::size_t>(a * 3 + b)] + L.mass * ((a == b ? dd : 0.0) - d[static_cast<std::size_t>(a)] * d[static_cast<std::size_t>(b)]);
    }
    m_inertia = I;
    for (const RotorModel& r : model.rotors) m_joints.push_back(r.joint);
}

void FlightController::hover() {
    m_mode = Mode::Hover;
    m_holdSet = false;
    m_yawRate = 0;
}
void FlightController::velocity(const V3d& v, double yawRate) {
    m_mode = Mode::Velocity;
    m_target = v;
    m_yawRate = yawRate;
}
void FlightController::moveTo(const V3d& p, double yawRate) {
    m_mode = Mode::Position;
    m_target = p;
    m_yawRate = yawRate;
}

void FlightController::update(Multibody& mb) {
    if (m_model.rotors.size() < 3) return;
    const double now = mb.time();
    const double dt = m_last < 0 ? mb.options().dt : now - m_last;
    m_last = now;
    const V3d p = mb.centreOfMass(), v = comVelocity(mb);
    const LinkPose base = mb.basePose();
    const auto& R = base.R;
    // Heading: the base's x axis seen from above.
    const V3d bx = rot(R, V3d{1, 0, 0});
    const double heading = std::atan2(bx[1], bx[0]);
    if (!m_yawSet) m_yaw = heading, m_yawSet = true;
    m_yaw += m_yawRate * dt;
    // Outer loops: position -> velocity -> acceleration.
    V3d vTarget = m_target;
    if (m_mode != Mode::Velocity) {
        if (m_mode == Mode::Hover && !m_holdSet) m_hold = p, m_holdSet = true;
        const V3d goal = m_mode == Mode::Hover ? m_hold : m_target;
        vTarget = mul(sub(goal, p), 1.5);
        const double s = norm(vTarget);
        if (s > 3.0) vTarget = mul(vTarget, 3.0 / s);
    }
    // Velocity loop: PI (the integral takes out what thrust and drag models leave over).
    const V3d ev = sub(vTarget, v);
    // (Only near the setpoint, so a long climb doesn't wind it up.)
    for (int k = 0; k < 3; ++k)
        if (std::fabs(ev[static_cast<std::size_t>(k)]) < 0.5)
            m_integral[static_cast<std::size_t>(k)] = std::clamp(m_integral[static_cast<std::size_t>(k)] + ev[static_cast<std::size_t>(k)] * dt, -2.0, 2.0);
    V3d a = add(mul(ev, 3.0), mul(m_integral, 1.5));
    const double horiz = std::hypot(a[0], a[1]), limit = 0.8 * kG;
    if (horiz > limit) a[0] *= limit / horiz, a[1] *= limit / horiz;
    a[2] = std::clamp(a[2], -0.6 * kG, 1.0 * kG);
    const V3d F = mul(sub(a, mb.options().gravity), m_mass); // the force the rotors must give
    const V3d zd = unit(F);
    // Desired attitude: thrust axis zd, heading m_yaw.
    const V3d xc{std::cos(m_yaw), std::sin(m_yaw), 0};
    const V3d yd = unit(cross(zd, xc));
    const V3d xd = cross(yd, zd);
    const std::array<double, 9> Rd = {xd[0], yd[0], zd[0], xd[1], yd[1], zd[1], xd[2], yd[2], zd[2]};
    // Attitude error e = 1/2 vee(Rd^T R - R^T Rd), in the body frame.
    std::array<double, 9> E{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s1 = 0, s2 = 0;
            for (int k = 0; k < 3; ++k) {
                s1 += Rd[static_cast<std::size_t>(k * 3 + i)] * R[static_cast<std::size_t>(k * 3 + j)];
                s2 += R[static_cast<std::size_t>(k * 3 + i)] * Rd[static_cast<std::size_t>(k * 3 + j)];
            }
            E[static_cast<std::size_t>(i * 3 + j)] = s1 - s2;
        }
    const V3d eR{0.5 * E[7], 0.5 * E[2], 0.5 * E[3]};
    const V3d wBody = rotT(R, mb.linkAngularVelocity(0));
    const V3d alpha = sub(mul(eR, -200.0), mul(wBody, 25.0)); // attitude loop at 14 rad/s: well above the velocity loop
    const V3d Iw = rot(m_inertia, wBody);
    const V3d tau = add(rot(m_inertia, alpha), cross(wBody, Iw));
    const double thrust = std::max(0.0, dot(F, rot(R, V3d{0, 0, 1})));
    // Mixer: thrust and three torques from the rotors' squared speeds (body frame).
    const std::size_t n = m_model.rotors.size();
    std::vector<std::array<double, 4>> A(n);
    for (std::size_t i = 0; i < n; ++i) {
        const RotorModel& r = m_model.rotors[i];
        const V3d arm = cross(r.offset, r.axisUp);
        A[i] = {r.kT * r.axisUp[2], r.kT * arm[0], r.kT * arm[1], r.kT * arm[2] - r.spin * r.kQ * r.axisUp[2]};
    }
    const double b[4] = {thrust, tau[0], tau[1], tau[2]};
    // Least squares, least norm: u = A^T (A A^T)^-1 b.
    double M[4][5] = {};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c)
            for (std::size_t i = 0; i < n; ++i) M[r][c] += A[i][static_cast<std::size_t>(r)] * A[i][static_cast<std::size_t>(c)];
        M[r][4] = b[r];
    }
    double y[4] = {0, 0, 0, 0};
    if (!solve4(M, y)) return;
    const double uMax = std::pow(2.5 * std::max(m_model.hoverSpeed, 1.0), 2);
    for (std::size_t i = 0; i < n; ++i) {
        double u = 0;
        for (int r = 0; r < 4; ++r) u += A[i][static_cast<std::size_t>(r)] * y[r];
        u = std::clamp(u, 0.0, uMax);
        const RotorModel& rm = m_model.rotors[i];
        JointControl c;
        c.mode = ActuatorMode::Velocity;
        c.target = rm.spin * std::sqrt(u) * rm.jointSign;
        mb.setControl(rm.joint, c);
    }
}

void DriveController::update(Multibody& mb) {
    // Skid steering scrubs: the wheels must ask for more turn than they get. A PI loop on the
    // measured yaw rate widens the speed difference until the turn is the one asked for.
    const double dt = mb.options().dt;
    const double yawRate = mb.linkAngularVelocity(0)[2];
    const double err = m_turn - yawRate;
    m_integral = std::clamp(m_integral + err * dt, -2.0, 2.0);
    const double turn = std::clamp(m_turn + 1.5 * err + 4.0 * m_integral, -6.0, 6.0);
    for (const WheelModel& w : m_model.wheels) {
        // Each wheel's ground speed for the turn, then its spin to roll at it.
        const double ground = m_speed - turn * w.lateral;
        JointControl c;
        c.mode = ActuatorMode::Velocity;
        c.target = ground / w.radius * w.jointSign;
        mb.setControl(w.joint, c);
    }
}

} // namespace sicdm

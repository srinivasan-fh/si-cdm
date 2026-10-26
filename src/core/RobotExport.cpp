#include "core/RobotExport.hpp"

#include "core/FileIO.hpp"
#include "core/Rig.hpp"
#include "core/Scene.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <array>
#include <functional>
#include <set>

namespace sicdm {

namespace fs = std::filesystem;

namespace {

std::string num(double v) {
    if (std::fabs(v) < 1e-12) v = 0;
    char b[40];
    std::snprintf(b, sizeof b, "%.9g", v);
    return b;
}

std::string vec(const std::array<double, 3>& v) { return num(v[0]) + " " + num(v[1]) + " " + num(v[2]); }

std::array<double, 3> sub(const std::array<double, 3>& a, const std::array<double, 3>& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }

std::string xmlEscape(const std::string& s) {
    std::string o;
    for (char c : s) {
        switch (c) {
        case '&': o += "&amp;"; break;
        case '<': o += "&lt;"; break;
        case '>': o += "&gt;"; break;
        case '"': o += "&quot;"; break;
        default: o += c;
        }
    }
    return o;
}

// Text inside an XML comment: escaped, and never "--" (which ends a comment's validity).
std::string commentText(const std::string& s) {
    std::string o = xmlEscape(s);
    for (std::size_t k = o.find("--"); k != std::string::npos; k = o.find("--", k)) o.replace(k, 2, "- -");
    return o;
}

// Names, frames and mesh files of a rig, worked out once for every format.
struct Plan {
    std::string model;
    std::vector<std::string> link, joint;
    std::vector<std::array<double, 3>> frame; // each link frame's origin (world, m)
    std::vector<std::string> mesh;            // file name in the mesh folder ("" none)
};

Plan plan(const RobotRig& rig) {
    Plan p;
    p.model = robotIdentifier(rig.name);
    // Unique ignoring case: the names become mesh files, and "Arm.stl" and "arm.stl" are one
    // file on Windows and macOS.
    std::set<std::string> used;
    auto folded = [](std::string n) {
        for (char& c : n) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return n;
    };
    auto unique = [&](std::string n) {
        std::string q = n;
        for (int k = 2; used.count(folded(q)); ++k) q = n + "_" + std::to_string(k);
        used.insert(folded(q));
        return q;
    };
    for (const RigLink& l : rig.links) {
        p.link.push_back(unique(robotIdentifier(l.name)));
        p.frame.push_back({0, 0, 0});
        p.mesh.push_back(l.bodies.empty() ? std::string() : p.link.back() + ".stl");
    }
    for (const RigJoint& j : rig.joints) {
        p.joint.push_back(unique(robotIdentifier(j.name)));
        p.frame[static_cast<std::size_t>(j.child)] = j.origin;
    }
    return p;
}

// The rig's joints by name, for mimics.
int jointIndex(const RobotRig& rig, const std::string& name) {
    for (std::size_t j = 0; j < rig.joints.size(); ++j)
        if (rig.joints[j].name == name) return static_cast<int>(j);
    return -1;
}

std::string urdf(const RobotRig& rig, const Plan& p, const std::string& meshDir) {
    std::string o = "<?xml version=\"1.0\"?>\n<!-- Written by SI CDM from the robot \"" + commentText(rig.name) + "\" (" +
                    morphologyKey(rig.morphology) + "). Lengths in metres, angles in radians. -->\n";
    o += "<robot name=\"" + p.model + "\">\n";
    if (!rig.floatingBase) o += "  <link name=\"world\"/>\n";
    for (std::size_t l = 0; l < rig.links.size(); ++l) {
        const RigLink& L = rig.links[l];
        const auto& I = L.inertia;
        o += "  <link name=\"" + p.link[l] + "\">\n";
        o += "    <inertial>\n      <origin xyz=\"" + vec(sub(L.com, p.frame[l])) + "\" rpy=\"0 0 0\"/>\n";
        o += "      <mass value=\"" + num(L.mass) + "\"/>\n";
        o += "      <inertia ixx=\"" + num(I[0]) + "\" ixy=\"" + num(I[1]) + "\" ixz=\"" + num(I[2]) + "\" iyy=\"" + num(I[4]) + "\" iyz=\"" + num(I[5]) +
             "\" izz=\"" + num(I[8]) + "\"/>\n    </inertial>\n";
        if (!p.mesh[l].empty())
            for (const char* kind : {"visual", "collision"})
                o += std::string("    <") + kind + ">\n      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n      <geometry><mesh filename=\"" + meshDir + "/" +
                     p.mesh[l] + "\"/></geometry>\n    </" + kind + ">\n";
        o += "  </link>\n";
    }
    if (!rig.floatingBase)
        o += "  <joint name=\"" + p.link[0] + "_to_world\" type=\"fixed\">\n    <parent link=\"world\"/>\n    <child link=\"" + p.link[0] +
             "\"/>\n    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n  </joint>\n";
    for (std::size_t j = 0; j < rig.joints.size(); ++j) {
        const RigJoint& J = rig.joints[j];
        o += "  <joint name=\"" + p.joint[j] + "\" type=\"" + rigJointTypeKey(J.type) + "\">\n";
        o += "    <parent link=\"" + p.link[static_cast<std::size_t>(J.parent)] + "\"/>\n    <child link=\"" + p.link[static_cast<std::size_t>(J.child)] + "\"/>\n";
        o += "    <origin xyz=\"" + vec(sub(p.frame[static_cast<std::size_t>(J.child)], p.frame[static_cast<std::size_t>(J.parent)])) + "\" rpy=\"0 0 0\"/>\n";
        o += "    <axis xyz=\"" + vec(J.axis) + "\"/>\n";
        const std::string ev = "effort=\"" + num(rigJointEffort(rig, j)) + "\" velocity=\"" + num(rigJointVelocity(rig, j)) + "\"";
        if (J.type == RigJointType::Continuous) o += "    <limit " + ev + "/>\n";
        else o += "    <limit lower=\"" + num(J.lo) + "\" upper=\"" + num(J.hi) + "\" " + ev + "/>\n";
        if (const int m = jointIndex(rig, J.mimic); m >= 0)
            o += "    <mimic joint=\"" + p.joint[static_cast<std::size_t>(m)] + "\" multiplier=\"" + num(J.mimicRatio) + "\" offset=\"0\"/>\n";
        o += "  </joint>\n";
    }
    o += "</robot>\n";
    return o;
}

std::string sdf(const RobotRig& rig, const Plan& p, const std::string& meshDir) {
    std::string o = "<?xml version=\"1.0\"?>\n<!-- Written by SI CDM from the robot \"" + commentText(rig.name) +
                    "\". Lengths in metres, angles in radians. Poses are in the model frame. -->\n";
    o += "<sdf version=\"1.7\">\n  <model name=\"" + p.model + "\">\n";
    for (std::size_t l = 0; l < rig.links.size(); ++l) {
        const RigLink& L = rig.links[l];
        const auto& I = L.inertia;
        o += "    <link name=\"" + p.link[l] + "\">\n      <pose>" + vec(p.frame[l]) + " 0 0 0</pose>\n";
        o += "      <inertial>\n        <pose>" + vec(sub(L.com, p.frame[l])) + " 0 0 0</pose>\n        <mass>" + num(L.mass) + "</mass>\n";
        o += "        <inertia><ixx>" + num(I[0]) + "</ixx><ixy>" + num(I[1]) + "</ixy><ixz>" + num(I[2]) + "</ixz><iyy>" + num(I[4]) + "</iyy><iyz>" +
             num(I[5]) + "</iyz><izz>" + num(I[8]) + "</izz></inertia>\n      </inertial>\n";
        if (!p.mesh[l].empty())
            for (const char* kind : {"visual", "collision"})
                o += std::string("      <") + kind + " name=\"" + p.link[l] + "_" + kind + "\">\n        <geometry><mesh><uri>" + meshDir + "/" + p.mesh[l] +
                     "</uri></mesh></geometry>\n      </" + kind + ">\n";
        o += "    </link>\n";
    }
    if (!rig.floatingBase)
        o += "    <joint name=\"" + p.link[0] + "_to_world\" type=\"fixed\">\n      <parent>world</parent>\n      <child>" + p.link[0] + "</child>\n    </joint>\n";
    for (std::size_t j = 0; j < rig.joints.size(); ++j) {
        const RigJoint& J = rig.joints[j];
        const bool slide = J.type == RigJointType::Prismatic;
        o += "    <joint name=\"" + p.joint[j] + "\" type=\"" + (slide ? "prismatic" : "revolute") + "\">\n";
        o += "      <parent>" + p.link[static_cast<std::size_t>(J.parent)] + "</parent>\n      <child>" + p.link[static_cast<std::size_t>(J.child)] + "</child>\n";
        o += "      <axis>\n        <xyz>" + vec(J.axis) + "</xyz>\n        <limit>\n";
        if (J.type == RigJointType::Continuous) o += "          <lower>-1e16</lower>\n          <upper>1e16</upper>\n";
        else o += "          <lower>" + num(J.lo) + "</lower>\n          <upper>" + num(J.hi) + "</upper>\n";
        o += "          <effort>" + num(rigJointEffort(rig, j)) + "</effort>\n          <velocity>" + num(rigJointVelocity(rig, j)) + "</velocity>\n";
        o += "        </limit>\n      </axis>\n";
        if (const int m = jointIndex(rig, J.mimic); m >= 0)
            o += "      <!-- coupled: " + num(J.mimicRatio) + " x " + p.joint[static_cast<std::size_t>(m)] + " (SDF 1.7 has no mimic joints) -->\n";
        o += "    </joint>\n";
    }
    o += "  </model>\n</sdf>\n";
    return o;
}

std::string mjcf(const RobotRig& rig, const Plan& p, const std::string& meshDir) {
    std::string o = "<!-- Written by SI CDM from the robot \"" + commentText(rig.name) + "\" (" + morphologyKey(rig.morphology) +
                    "). Lengths in metres, angles in radians. -->\n";
    o += "<mujoco model=\"" + p.model + "\">\n";
    o += "  <compiler angle=\"radian\" meshdir=\"" + meshDir + "\" inertiafromgeom=\"false\" autolimits=\"true\"/>\n";
    o += "  <option timestep=\"0.002\" gravity=\"0 0 -9.80665\"/>\n  <asset>\n";
    for (std::size_t l = 0; l < rig.links.size(); ++l)
        if (!p.mesh[l].empty()) o += "    <mesh name=\"" + p.link[l] + "_mesh\" file=\"" + p.mesh[l] + "\"/>\n";
    o += "  </asset>\n  <worldbody>\n    <light pos=\"0 0 3\" dir=\"0 0 -1\"/>\n";
    o += "    <geom name=\"floor\" type=\"plane\" size=\"0 0 0.05\" rgba=\"0.8 0.8 0.8 1\"/>\n";
    // Children of each link, nested.
    std::vector<std::vector<std::size_t>> kids(rig.links.size());
    for (std::size_t j = 0; j < rig.joints.size(); ++j) kids[static_cast<std::size_t>(rig.joints[j].parent)].push_back(j);
    std::function<void(std::size_t, int, const std::string&)> body = [&](std::size_t l, int viaJoint, const std::string& ind) {
        const RigLink& L = rig.links[l];
        const std::array<double, 3> parentFrame =
            viaJoint < 0 ? std::array<double, 3>{0, 0, 0} : p.frame[static_cast<std::size_t>(rig.joints[static_cast<std::size_t>(viaJoint)].parent)];
        o += ind + "<body name=\"" + p.link[l] + "\" pos=\"" + vec(sub(p.frame[l], parentFrame)) + "\">\n";
        if (viaJoint < 0 && rig.floatingBase) o += ind + "  <freejoint name=\"" + p.link[l] + "_free\"/>\n";
        if (viaJoint >= 0) {
            const RigJoint& J = rig.joints[static_cast<std::size_t>(viaJoint)];
            o += ind + "  <joint name=\"" + p.joint[static_cast<std::size_t>(viaJoint)] + "\" type=\"" +
                 (J.type == RigJointType::Prismatic ? "slide" : "hinge") + "\" axis=\"" + vec(J.axis) + "\"";
            if (J.type != RigJointType::Continuous) o += " range=\"" + num(J.lo) + " " + num(J.hi) + "\"";
            o += " damping=\"0.05\"/>\n";
        }
        const auto& I = L.inertia;
        o += ind + "  <inertial pos=\"" + vec(sub(L.com, p.frame[l])) + "\" mass=\"" + num(L.mass) + "\" fullinertia=\"" + num(I[0]) + " " + num(I[4]) + " " +
             num(I[8]) + " " + num(I[1]) + " " + num(I[2]) + " " + num(I[5]) + "\"/>\n";
        if (!p.mesh[l].empty()) o += ind + "  <geom type=\"mesh\" mesh=\"" + p.link[l] + "_mesh\"/>\n";
        for (std::size_t j : kids[l]) body(static_cast<std::size_t>(rig.joints[j].child), static_cast<int>(j), ind + "  ");
        o += ind + "</body>\n";
    };
    body(0, -1, "    ");
    o += "  </worldbody>\n";
    // Couplings and actuators.
    std::string eq, act;
    for (std::size_t j = 0; j < rig.joints.size(); ++j) {
        const RigJoint& J = rig.joints[j];
        if (const int m = jointIndex(rig, J.mimic); m >= 0) {
            eq += "    <joint joint1=\"" + p.joint[j] + "\" joint2=\"" + p.joint[static_cast<std::size_t>(m)] + "\" polycoef=\"0 " + num(J.mimicRatio) + " 0 0 0\"/>\n";
            continue;
        }
        const double effort = rigJointEffort(rig, j);
        if (J.type == RigJointType::Continuous)
            act += "    <velocity name=\"" + p.joint[j] + "_speed\" joint=\"" + p.joint[j] + "\" kv=\"" + num(effort) + "\" forcerange=\"" + num(-effort) + " " +
                   num(effort) + "\"/>\n";
        else
            act += "    <position name=\"" + p.joint[j] + "_servo\" joint=\"" + p.joint[j] + "\" kp=\"" + num(effort * 10.0) + "\" forcerange=\"" + num(-effort) +
                   " " + num(effort) + "\" ctrlrange=\"" + num(J.lo) + " " + num(J.hi) + "\"/>\n";
    }
    if (!eq.empty()) o += "  <equality>\n" + eq + "  </equality>\n";
    if (!act.empty()) o += "  <actuator>\n" + act + "  </actuator>\n";
    o += "</mujoco>\n";
    return o;
}

void appendF32(std::string& s, float v) {
    char b[4];
    std::memcpy(b, &v, 4);
    s.append(b, 4);
}

// A link's parts as one binary STL in metres, in the link's frame.
std::string linkStl(const Scene& scene, const RigLink& link, const std::array<double, 3>& frame) {
    std::string data(80, '\0');
    const char header[] = "SI CDM robot link (m)";
    std::memcpy(data.data(), header, sizeof(header) - 1);
    std::string tris;
    std::uint32_t count = 0;
    for (std::size_t b : link.bodies) {
        const Body& body = scene.bodies()[b];
        const auto& v = body.mesh.vertices;
        const auto& idx = body.mesh.indices;
        for (std::size_t t = 0; t + 2 < idx.size(); t += 3) {
            Vec3 p[3];
            for (int k = 0; k < 3; ++k) {
                const Vec3 w = v[idx[t + static_cast<std::size_t>(k)]].position + body.offset;
                p[k] = {static_cast<float>(w.x * 1e-3 - frame[0]), static_cast<float>(w.y * 1e-3 - frame[1]), static_cast<float>(w.z * 1e-3 - frame[2])};
            }
            const Vec3 n = normalize(cross(p[1] - p[0], p[2] - p[0]));
            for (const Vec3& q : {n, p[0], p[1], p[2]}) {
                appendF32(tris, q.x);
                appendF32(tris, q.y);
                appendF32(tris, q.z);
            }
            tris.append(2, '\0');
            ++count;
        }
    }
    char cnt[4];
    std::memcpy(cnt, &count, 4);
    data.append(cnt, 4);
    return data + tris;
}

} // namespace

bool robotFormatOf(const std::string& path, RobotFormat& out) {
    std::string ext = fs::path(path).extension().string();
    for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (ext == ".urdf") out = RobotFormat::Urdf;
    else if (ext == ".sdf") out = RobotFormat::Sdf;
    else if (ext == ".xml" || ext == ".mjcf") out = RobotFormat::Mjcf;
    else return false;
    return true;
}

const char* robotFormatName(RobotFormat f) {
    switch (f) {
    case RobotFormat::Urdf: return "URDF";
    case RobotFormat::Sdf: return "SDF";
    case RobotFormat::Mjcf: return "MJCF";
    }
    return "URDF";
}

std::string robotIdentifier(const std::string& name) {
    std::string o;
    for (char c : name) o += std::isalnum(static_cast<unsigned char>(c)) ? c : '_';
    if (o.empty() || std::isdigit(static_cast<unsigned char>(o[0]))) o = "r_" + o;
    return o;
}

std::string robotDescription(const Scene& scene, const RobotRig& rig, RobotFormat format, const std::string& meshDir) {
    (void)scene;
    const Plan p = plan(rig);
    switch (format) {
    case RobotFormat::Urdf: return urdf(rig, p, meshDir);
    case RobotFormat::Sdf: return sdf(rig, p, meshDir);
    case RobotFormat::Mjcf: return mjcf(rig, p, meshDir);
    }
    return {};
}

RobotExportResult exportRobot(const Scene& scene, const RobotRig& rig, const std::string& path) {
    RobotExportResult r;
    RobotFormat format;
    if (!robotFormatOf(path, format)) {
        r.error = "Robot files are .urdf, .sdf or .xml (MJCF)";
        return r;
    }
    const fs::path file(path);
    const std::string meshDir = file.stem().string() + "_meshes";
    const fs::path dir = file.parent_path() / meshDir;
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) {
        r.error = "Can't make " + dir.string() + ": " + ec.message();
        return r;
    }
    if (!writeFile(path, robotDescription(scene, rig, format, meshDir), &r.error)) return r;
    r.files.push_back(path);
    const Plan p = plan(rig);
    for (std::size_t l = 0; l < rig.links.size(); ++l) {
        if (p.mesh[l].empty()) continue;
        const std::string m = (dir / p.mesh[l]).string();
        if (!writeFile(m, linkStl(scene, rig.links[l], p.frame[l]), &r.error)) return r;
        r.files.push_back(m);
    }
    r.ok = true;
    return r;
}

} // namespace sicdm

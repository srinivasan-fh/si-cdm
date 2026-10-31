#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/PointCloud.hpp"
#include "core/Print.hpp"
#include "core/Sculpt.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace sicdm;

#define REQUIRE(c)                                    \
    do {                                              \
        const bool required_ = static_cast<bool>(c); \
        CHECK(required_);                             \
        if (!required_) return;                       \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }
std::string tmp(const char* n) { return (std::filesystem::temp_directory_path() / n).string(); }

// Points evenly over a sphere (a Fibonacci lattice), centred at c.
std::vector<Vec3> spherePoints(int n, float r, Vec3 c) {
    std::vector<Vec3> p;
    const float golden = 3.14159265f * (3.0f - std::sqrt(5.0f));
    for (int i = 0; i < n; ++i) {
        const float z = 1.0f - 2.0f * (static_cast<float>(i) + 0.5f) / static_cast<float>(n), rr = std::sqrt(1 - z * z), t = golden * static_cast<float>(i);
        p.push_back(c + Vec3{rr * std::cos(t), rr * std::sin(t), z} * r);
    }
    return p;
}

} // namespace

TEST_CASE("Point clouds: normals, spacing and a mesh from a sampled sphere") {
    const std::vector<Vec3> pts = spherePoints(6000, 20, {5, -3, 30});
    const float sp = pointSpacing(pts);
    CHECK(sp > 0.5f && sp < 1.5f); // about sqrt(4 pi 400 / 6000) = 0.92
    // Normals estimated and turned one way: all out of the sphere.
    const std::vector<Vec3> n = estimateNormals(pts);
    int out = 0;
    for (std::size_t i = 0; i < pts.size(); ++i) out += dot(n[i], normalize(pts[i] - Vec3{5, -3, 30})) > 0.95f;
    CHECK(out == static_cast<int>(pts.size()));
    // The mesh: closed, round, the sphere's size.
    ReconstructReport rep;
    std::string why;
    const MeshData m = reconstructSurface(pts, {}, 0, &rep, &why);
    REQUIRE(!m.indices.empty());
    Body b = meshBody("s", m);
    const PrintReport pr = analyzePrint(b);
    CHECK(pr.openEdges == 0 && pr.nonManifoldEdges == 0 && !pr.insideOut);
    const double v = meshVolume(m), want = 4.0 / 3 * 3.14159265 * 8000;
    CHECK(std::fabs(v - want) < 0.03 * want);
    float worst = 0;
    for (const Vertex& q : m.vertices) worst = std::max(worst, std::fabs(length(q.position - Vec3{5, -3, 30}) - 20.0f));
    CHECK(worst < 0.6f);
}

TEST_CASE("Point clouds: XYZ and PLY files, the commands, saved and opened") {
    const std::vector<Vec3> pts = spherePoints(3000, 15, {0, 0, 15});
    const std::string xyz = tmp("sicdm_cloud.xyz"), ply = tmp("sicdm_cloud.ply");
    {
        std::ofstream f(xyz);
        f << "# a scan\n" << pts.size() << "\n";
        for (const Vec3& p : pts) f << p.x << "," << p.y << "," << p.z << "\n";
        std::ofstream g(ply, std::ios::binary);
        g << "ply\nformat binary_little_endian 1.0\nelement vertex " << pts.size() << "\nproperty float x\nproperty float y\nproperty float z\n"
          << "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
        for (const Vec3& p : pts) {
            g.write(reinterpret_cast<const char*>(&p.x), 4).write(reinterpret_cast<const char*>(&p.y), 4).write(reinterpret_cast<const char*>(&p.z), 4);
            const unsigned char rgb[3] = {200, 100, 50};
            g.write(reinterpret_cast<const char*>(rgb), 3);
        }
    }
    std::vector<Vec3> a, an, b, bn;
    REQUIRE(readPointCloud(xyz, a, an));
    REQUIRE(readPointCloud(ply, b, bn));
    CHECK(a.size() == pts.size() && b.size() == pts.size() && an.empty() && bn.empty());
    CHECK(length(b[17] - pts[17]) < 1e-6f && length(a[17] - pts[17]) < 1e-3f);

    Document d;
    REQUIRE(run(d, "IMPORT " + xyz).ok); // by its extension
    CHECK(d.scene.bodies().back().kind == "pointcloud" && d.scene.bodies().back().mesh.vertices.size() == pts.size());
    REQUIRE(run(d, "POINTCLOUD IMPORT " + ply + " SCALE 2 @100,0,0").ok);
    const std::string cloud = d.scene.bodies().back().name;
    CHECK(std::fabs(d.scene.bodies().back().mesh.bounds().max.z - 60) < 0.1f);
    REQUIRE(run(d, "POINTCLOUD NORMALS \"" + cloud + "\"").ok);
    REQUIRE(run(d, "POINTCLOUD MESH \"" + cloud + "\"").ok);
    const Body& mesh = d.scene.bodies().back();
    CHECK(mesh.kind == "mesh" && mesh.offset.x == 100.0f);
    CHECK(std::fabs(meshVolume(mesh.mesh) - 4.0 / 3 * 3.14159265 * 27000) < 0.03 * 4.0 / 3 * 3.14159265 * 27000);
    CHECK(!d.scene.bodies()[d.scene.bodies().size() - 2].visible); // the cloud hidden
    CHECK(!run(d, "POINTCLOUD MESH \"" + mesh.name + "\"").ok);    // not a cloud

    // Saved and opened: the points come back.
    const std::string proj = tmp("sicdm_cloud.sicdm");
    REQUIRE(run(d, "SAVE " + proj).ok);
    Document e;
    REQUIRE(run(e, "OPEN " + proj).ok);
    bool found = false;
    for (const Body& x : e.scene.bodies())
        if (x.kind == "pointcloud" && x.mesh.vertices.size() == pts.size()) found = true;
    CHECK(found);
}

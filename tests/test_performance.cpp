#include "TestFramework.hpp"

#include "core/Brep.hpp"
#include "core/Bvh.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Json.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

constexpr double kPiP = 3.14159265358979323846;

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }
std::string tempPath(const char* name) { return (std::filesystem::temp_directory_path() / name).string(); }

#if defined(NDEBUG) && !defined(__SANITIZE_ADDRESS__)
constexpr double kSlack = 1.0; // optimised builds: the real limits
#else
constexpr double kSlack = 15.0; // debug and sanitizer builds run many times slower
#endif

double msSince(std::chrono::steady_clock::time_point t) { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count(); }

// A flat square plate face, side `side`, with n x n round holes of radius r through it
// (one face: the sort that slowed tessellation down to seconds).
Brep perforatedFace(int n, double side, double r) {
    Brep b;
    BSurface plane;
    plane.kind = BSurfaceKind::Plane;
    b.surfaces.push_back(plane);
    BFace f;
    f.surface = 0;
    // Outer square.
    BLoop outer;
    outer.outer = true;
    const DVec3 c[4] = {{0, 0, 0}, {side, 0, 0}, {side, side, 0}, {0, side, 0}};
    for (const DVec3& p : c) b.vertices.push_back({p});
    for (int k = 0; k < 4; ++k) {
        b.edges.push_back({k, (k + 1) % 4, -1, true});
        outer.edges.push_back({k, true});
    }
    f.loops.push_back(outer);
    // Holes: a closed circle each, run clockwise.
    const double pitch = side / n;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            const DVec3 centre{pitch * (i + 0.5), pitch * (j + 0.5), 0};
            BCurve circle;
            circle.kind = BCurveKind::Circle;
            circle.f = Frame::make(centre, {0, 0, 1}, {1, 0, 0});
            circle.r1 = r;
            b.curves.push_back(circle);
            b.vertices.push_back({centre + DVec3{r, 0, 0}});
            const int v = static_cast<int>(b.vertices.size()) - 1;
            b.edges.push_back({v, v, static_cast<int>(b.curves.size()) - 1, false});
            BLoop hole;
            hole.edges.push_back({static_cast<int>(b.edges.size()) - 1, true});
            f.loops.push_back(hole);
        }
    BShell sh;
    sh.closed = false;
    sh.faces.push_back(f);
    b.shells.push_back(sh);
    orientLoops(b);
    return b;
}

double meshArea(const MeshData& m) {
    double a = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const DVec3 p(m.vertices[m.indices[t]].position), q(m.vertices[m.indices[t + 1]].position), s(m.vertices[m.indices[t + 2]].position);
        a += 0.5 * dlength(dcross(q - p, s - p));
    }
    return a;
}

// A bumpy closed mesh of some 140k triangles.
MeshData bigMesh() {
    Document d;
    run(d, "SPHERE 50");
    for (int k = 0; k < 4; ++k) run(d, "SUBDIVIDE \"Sphere 1\"");
    return d.scene.bodies()[0].mesh;
}

} // namespace

TEST_CASE("Performance: a face with 900 holes tessellates in well under a second, covering the right area") {
    const Brep b = perforatedFace(30, 300.0, 3.0);
    TessellateOptions o;
    o.chord = 0.05;
    const auto t = std::chrono::steady_clock::now();
    TessellateReport rep;
    const MeshData m = tessellate(b, o, &rep);
    const double ms = msSince(t);
    CHECK(rep.failedFaces == 0);
    CHECK(ms < 1000 * kSlack);
    // The plate less its holes: the holes' polygons sit inside the circles, so a little
    // more than the exact area, but not more than a few percent of the holes' area.
    const double holes = 900 * kPiP * 9.0, area = meshArea(m);
    CHECK(area > 300.0 * 300.0 - holes);
    CHECK(area < 300.0 * 300.0 - 0.97 * holes);
    // Meshing again gives the very same mesh (the faces are meshed side by side).
    CHECK(meshContentKey(tessellate(b, o)) == meshContentKey(m));
}

TEST_CASE("Performance: the picking BVH finds the same hits as testing every triangle") {
    const MeshData m = bigMesh();
    REQUIRE(m.indices.size() / 3 > 100000);
    const MeshBvh bvh(m);
    CHECK(bvh.triangles() == m.indices.size() / 3);
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> U(-60, 60);
    int hits = 0, same = 0;
    const int rays = 300;
    for (int k = 0; k < rays; ++k) {
        const Vec3 from{U(rng), U(rng), 200};
        const Ray r{from, normalize(Vec3{U(rng), U(rng), 50} - from)};
        float best = 1e30f;
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
            float h = 0;
            if (intersectTriangle(r, m.vertices[m.indices[t]].position, m.vertices[m.indices[t + 1]].position, m.vertices[m.indices[t + 2]].position, h) &&
                h < best)
                best = h;
        }
        float tb = 1e30f;
        const bool hit = bvh.raycast(r, tb);
        hits += hit;
        same += hit == (best < 1e30f) && (!hit || std::fabs(tb - best) < 1e-4f);
    }
    CHECK(hits > rays / 4);
    CHECK(same == rays);
}

TEST_CASE("Performance: scene picking builds its BVH once the scene holds still, and stays right after edits") {
    Document d;
    run(d, "SPHERE 50");
    for (int k = 0; k < 3; ++k) run(d, "SUBDIVIDE \"Sphere 1\"");
    run(d, "BOX 20 20 20 @150,0,0");
    const Ray down{{0, 0, 300}, {0, 0, -1}}, box{{150, 0, 300}, {0, 0, -1}};
    // The first cast at a revision tests every triangle; the second builds; then it is quick.
    auto a = d.scene.raycast(down);
    auto b = d.scene.raycast(down);
    REQUIRE(a && b);
    CHECK(a->body == 0 && b->body == 0);
    CHECK(std::fabs(a->t - b->t) < 1e-4f);
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < 1000; ++k) b = d.scene.raycast(down);
    CHECK(msSince(t0) < 100 * kSlack); // 0.1 ms a ray at most
    auto c = d.scene.raycast(box);
    REQUIRE(c.has_value());
    CHECK(c->body == 1);
    CHECK(std::fabs(c->point.z - 20) < 1e-3f);
    // A change to the sphere: the old tree is not used.
    run(d, "SCALE \"Sphere 1\" 2");
    for (int k = 0; k < 3; ++k) {
        const auto h = d.scene.raycast(down);
        REQUIRE(h.has_value());
        CHECK(std::fabs(h->point.z - 200) < 0.5f); // the doubled sphere's top
    }
}

TEST_CASE("Performance: coarser copies keep the shape, its winding, and lose most triangles") {
    const MeshData m = bigMesh();
    const AABB box = m.bounds();
    const float diag = length(box.max - box.min);
    const MeshData c = clusterMesh(m, diag / 40);
    CHECK(c.indices.size() * 10 < m.indices.size()); // a tenth or fewer
    CHECK(!c.indices.empty());
    CHECK(c.edgeVertices.empty());
    const AABB cb = c.bounds();
    CHECK(length(cb.min - box.min) < diag / 20);
    CHECK(length(cb.max - box.max) < diag / 20);
    // Facing out still: the volume is positive and near the sphere's.
    const double v0 = meshVolume(m), v1 = meshVolume(c);
    CHECK(v1 > 0.85 * v0);
    CHECK(v1 < 1.1 * v0);
    for (std::size_t t = 0; t + 2 < c.indices.size(); t += 3) {
        CHECK(c.indices[t] != c.indices[t + 1]);
        CHECK(c.indices[t + 1] != c.indices[t + 2]);
    }
    // Content keys: equal for equal meshes, different once a vertex moves.
    MeshData copy = m;
    CHECK(meshContentKey(copy) == meshContentKey(m));
    copy.vertices[123].position.x += 1e-3f;
    CHECK(meshContentKey(copy) != meshContentKey(m));
}

TEST_CASE("Performance: pattern copies are the body itself moved, exact and quick") {
    Document d;
    run(d, "BOX 30 20 10 @0,0,0");
    run(d, "SPHERE 50 @0,200,0");
    for (int k = 0; k < 4; ++k) run(d, "SUBDIVIDE \"Sphere 2\"");
    const auto t0 = std::chrono::steady_clock::now();
    REQUIRE(run(d, "PATTERN \"Sphere 2\" LINEAR 5 SPACING 120 DIR X ROWS 4 SPACING2 120 DIR2 Y").ok);
    CHECK(msSince(t0) < 1000 * kSlack);
    CHECK(d.scene.bodies().size() == 21);
    const Body& s0 = d.scene.bodies()[1];
    const Body& s7 = d.scene.bodies()[8];
    CHECK(meshContentKey(s7.mesh) == meshContentKey(s0.mesh)); // shifted, not remeshed
    // Round an axis: a block stays a block (and exact), turned.
    REQUIRE(run(d, "PATTERN \"Block 1\" CIRCULAR 4 AXIS Z AT 0,-100,0").ok);
    const Body* q = nullptr;
    for (const Body& b : d.scene.bodies())
        if (b.name == "Block 1 (3)") q = &b; // half way round
    REQUIRE(q != nullptr);
    CHECK(q->kind == "box");
    CHECK(q->exact());
    CHECK(std::fabs(std::fabs(q->rotation.z) - 180) < 1e-4);
    const AABB qb = q->bounds();
    CHECK(std::fabs((qb.min.y + qb.max.y) / 2 - (-200)) < 1e-3);
    CHECK(std::fabs(qb.max.x - qb.min.x - 30) < 1e-3);
}

TEST_CASE("Performance: big meshes are saved as binary once per shape, and read back the same") {
    Document d;
    run(d, "SPHERE 50");
    for (int k = 0; k < 3; ++k) run(d, "SUBDIVIDE \"Sphere 1\"");
    REQUIRE(d.scene.bodies()[0].mesh.vertices.size() >= 20000);
    run(d, "PATTERN \"Sphere 1\" LINEAR 3 SPACING 150 DIR X");
    run(d, "BOX 10 10 10 @0,300,0");
    const std::string path = tempPath("sicdm_big.sicdm");
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    std::stringstream ss;
    ss << std::ifstream(path).rdbuf();
    const std::string text = ss.str();
    CHECK(text.find("positions_b64") != std::string::npos);
    CHECK(text.find("mesh_same_as") != std::string::npos);
    const auto j = Json::parse(text);
    REQUIRE(j.has_value());
    CHECK((*j)["reader_version"].asInt() == 5);
    Scene s;
    ProjectState st;
    REQUIRE(loadProject(s, st, path, &err));
    REQUIRE(s.bodies().size() == 4);
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK(meshContentKey(s.bodies()[i].mesh) == meshContentKey(d.scene.bodies()[i].mesh));
        CHECK(s.bodies()[i].name == d.scene.bodies()[i].name);
        CHECK(std::fabs(s.bodies()[i].offset.x - d.scene.bodies()[i].offset.x) < 1e-6f);
    }
    std::filesystem::remove(path);
    // A small project stays readable by older versions.
    Document e;
    run(e, "BOX 10 10 10");
    REQUIRE(saveProject(e.scene, e.state, path, &err));
    std::stringstream s2;
    s2 << std::ifstream(path).rdbuf();
    const auto j2 = Json::parse(s2.str());
    REQUIRE(j2.has_value());
    CHECK((*j2)["reader_version"].asInt() == 1);
    std::filesystem::remove(path);
}

TEST_CASE("Performance: undo keeps unchanged meshes once, so a big model keeps its whole undo history") {
    Document d;
    REQUIRE(run(d, "SPHERE 20 @0,0,0").ok);
    for (int i = 0; i < 3; ++i) REQUIRE(run(d, "SUBDIVIDE \"Sphere 1\"").ok);
    REQUIRE(run(d, "PATTERN \"Sphere 1\" LINEAR 32 SPACING 50").ok);
    std::size_t vertices = 0;
    for (const Body& b : d.scene.bodies()) vertices += b.mesh.vertices.size();
    REQUIRE(vertices > 2000000); // more than the undo history's budget, kept as copies
    const std::uint64_t shape = meshContentKey(d.scene.bodies()[0].mesh);
    // Ten moves of different parts: all ten undo, back to where each was.
    for (int i = 0; i < 10; ++i) REQUIRE(run(d, "MOVE 0,5,0 \"Sphere 1 (" + std::to_string(i + 2) + ")\"").ok);
    for (int i = 0; i < 10; ++i) REQUIRE(run(d, "UNDO").ok);
    for (const Body& b : d.scene.bodies()) CHECK(std::fabs(b.offset.y) < 1e-6f);
    // A reshaped part comes back as it was, the others untouched.
    REQUIRE(run(d, "SCULPT \"Sphere 1\" INFLATE AT 0,0,40 RADIUS 8").ok);
    CHECK(meshContentKey(d.scene.bodies()[0].mesh) != shape);
    REQUIRE(run(d, "MOVE 5,0,0 \"Sphere 1 (3)\"").ok);
    REQUIRE(run(d, "UNDO").ok);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(meshContentKey(d.scene.bodies()[0].mesh) == shape);
    REQUIRE(run(d, "REDO").ok);
    CHECK(meshContentKey(d.scene.bodies()[0].mesh) != shape);
    // Thousands of pattern copies get their names quickly.
    Document e;
    REQUIRE(run(e, "CYLINDER 3 10 @0,0,0").ok);
    const auto t0 = std::chrono::steady_clock::now();
    REQUIRE(run(e, "PATTERN \"Cylinder 1\" LINEAR 60 SPACING 10 ROWS 60 SPACING2 10 DIR2 0,1,0").ok);
    CHECK(e.scene.bodies().size() == 3600);
    CHECK(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < 20.0); // was quadratic
    CHECK(e.scene.bodies().back().name == "Cylinder 1 (3600)");
}

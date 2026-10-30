#include "TestFramework.hpp"

#include "core/Armature.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Json.hpp"
#include "core/Mcp.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"

#include <cmath>
#include <filesystem>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

constexpr double kPiD = 3.14159265358979323846;

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

Body* named(Document& d, const std::string& n) {
    for (Body& b : d.scene.bodies())
        if (b.name == n) return &b;
    return nullptr;
}

Armature armatureOf(Document& d, const std::string& n) {
    Armature a;
    if (Body* b = named(d, n)) armatureFromParams(b->params, a);
    return a;
}

std::string joined(const CommandOutput& o) {
    std::string s;
    for (const std::string& l : o.lines) s += l + "\n";
    return s;
}

} // namespace

TEST_CASE("armature: params, a sound tree, quaternions that agree with the Euler turns") {
    Armature a;
    a.balls = {{{0, 0, 0}, 10, -1, {0, 0, 0, 1}}, {{50, 0, 0}, 8, 0, {0, 0, 0, 1}}, {{100, 0, 0}, 6, 1, {0, 0, 0, 1}}};
    Armature back;
    REQUIRE(armatureFromParams(armatureParams(a), back));
    CHECK(back.balls.size() == 3 && back.balls[2].parent == 1 && back.balls[1].radius == 8);
    std::vector<float> loop = armatureParams(a);
    loop[4] = 2; // the root on ball 3: no root left, and a loop
    CHECK(!armatureFromParams(loop, back));
    std::vector<float> flat = armatureParams(a);
    flat[3 * kArmatureFloats - kArmatureFloats + 3] = 0; // a ball without size
    CHECK(!armatureFromParams(flat, back));
    CHECK(!armatureFromParams({1, 2, 3}, back));
    // The quaternion of an Euler turn is that turn; and back.
    const Vec3 e{20, -35, 70};
    const Mat4 m = quatMatrix(quatFromEuler(e)), r = eulerRotation(e);
    for (int c = 0; c < 3; ++c)
        for (int k = 0; k < 3; ++k) CHECK_NEAR(m.m[c][k], r.m[c][k], 1e-5);
    const Vec3 again = quatToEuler(quatFromEuler(e));
    CHECK_NEAR(again.x, 20, 1e-3);
    CHECK_NEAR(again.y, -35, 1e-3);
    CHECK_NEAR(again.z, 70, 1e-3);
    // Posing: ball 2 turned 90 degrees about Z swings ball 3 round it.
    a.balls[1].turn = quatFromAxisAngle({0, 0, 1}, radians(90));
    const std::vector<Vec3> p = posedBalls(a);
    CHECK(length(p[1] - Vec3{50, 0, 0}) < 1e-4f);
    CHECK(length(p[2] - Vec3{50, 50, 0}) < 1e-4f);
}

TEST_CASE("armature skin: a ball's skin is the sphere, two balls' the capsule, closed and bound") {
    Armature one;
    one.balls = {{{0, 0, 0}, 20, -1, {0, 0, 0, 1}}};
    MeshData m;
    SkinData s;
    REQUIRE(armatureSkin(one, SkinOptions{}, m, s));
    CHECK(openEdgeCount(m) == 0);
    CHECK_NEAR(meshVolume(m), 4.0 / 3 * kPiD * 8000, 0.01 * 4.0 / 3 * kPiD * 8000); // on the sphere, its facets' chords short of it
    double worst = 0;
    for (const Vertex& v : m.vertices) worst = std::max(worst, std::fabs(static_cast<double>(length(v.position)) - 20.0));
    CHECK(worst < 0.01); // every vertex on the sphere
    CHECK(s.rest.size() == m.vertices.size());
    CHECK(s.bone[0][0] == -1); // one ball: nothing to follow

    Armature two;
    two.balls = {{{0, 0, 0}, 10, -1, {0, 0, 0, 1}}, {{60, 0, 0}, 10, 0, {0, 0, 0, 1}}};
    SkinOptions o;
    o.blend = 0;
    REQUIRE(armatureSkin(two, o, m, s));
    CHECK(openEdgeCount(m) == 0);
    const double capsule = kPiD * 100 * 60 + 4.0 / 3 * kPiD * 1000;
    CHECK_NEAR(meshVolume(m), capsule, 0.01 * capsule);
    for (std::size_t i = 0; i < s.bone.size(); ++i) {
        CHECK(s.bone[i][0] == 1); // all on the one link
        CHECK_NEAR(s.weight[i][0], 1.0, 1e-6);
    }
}

TEST_CASE("armature skin of a figure: closed, no triangle facing in, even where the voxels fall on its balls") {
    // Balls on round numbers put voxel samples exactly on the surface (4^2 + 1^2 + 8^2 = 9^2 at
    // the feet): the marching once made slivers facing anywhere there.
    Document d;
    for (const char* c : {"ARMATURE NEW Body AT 0,0,100 RADIUS 20", "ARMATURE Body ADD AT 0,0,140 RADIUS 14", "ARMATURE Body ADD TO 1 AT 40,0,100 RADIUS 10 MIRROR X",
                          "ARMATURE Body ADD TO 3 AT 80,0,100 RADIUS 8 MIRROR X", "ARMATURE Body ADD TO 1 AT 15,0,60 RADIUS 12 MIRROR X",
                          "ARMATURE Body ADD TO 7 AT 15,0,10 RADIUS 9 MIRROR X", "ARMATURE Body SKIN"})
        REQUIRE(run(d, c).ok);
    const Body* s = named(d, "Body skin");
    REQUIRE(s != nullptr);
    CHECK(openEdgeCount(s->mesh) == 0);
    int facingIn = 0;
    const MeshData& m = s->mesh;
    for (std::size_t k = 0; k + 2 < m.indices.size(); k += 3) {
        const Vertex &a = m.vertices[m.indices[k]], &b = m.vertices[m.indices[k + 1]], &c = m.vertices[m.indices[k + 2]];
        if (dot(cross(b.position - a.position, c.position - a.position), a.normal + b.normal + c.normal) <= 0) ++facingIn;
    }
    CHECK(facingIn == 0);
    CHECK(meshVolume(m) > 0);
}

TEST_CASE("armature pose bends the skin: the far end turns with the ball, the near end stays, edits are kept") {
    Document d;
    REQUIRE(run(d, "ARMATURE NEW Arm AT 0,0,0 RADIUS 10").ok);
    REQUIRE(run(d, "ARMATURE Arm ADD AT 50,0,0 RADIUS 8").ok);
    REQUIRE(run(d, "ARMATURE Arm ADD AT 100,0,0 RADIUS 6").ok);
    REQUIRE(run(d, "ARMATURE Arm SKIN").ok);
    Body* skin = named(d, "Arm skin");
    REQUIRE(skin && skin->skin);
    CHECK(!named(d, "Arm")->visible); // the skin is what is seen
    const SkinData restSkin = *skin->skin;
    REQUIRE(run(d, "ARMATURE Arm POSE 2 AXIS 0,0,1 ANGLE 90").ok);
    skin = named(d, "Arm skin");
    const Mat4 turn = translate({50, 0, 0}) * rotateAxis({0, 0, 1}, radians(90)) * translate({-50, 0, 0});
    double farWorst = 0, nearWorst = 0;
    for (std::size_t i = 0; i < restSkin.rest.size(); ++i) {
        const Vec3 r = restSkin.rest[i], now = skin->mesh.vertices[i].position;
        if (r.x > 88) farWorst = std::max(farWorst, static_cast<double>(length(now - turn.transformPoint(r))));
        if (r.x < 12) nearWorst = std::max(nearWorst, static_cast<double>(length(now - r)));
    }
    CHECK(farWorst < 0.05);
    CHECK(nearWorst < 0.05);
    CHECK(openEdgeCount(skin->mesh) == 0);
    // An edit to the posed skin (a vertex pulled out) is kept through the rest pose and back.
    std::size_t tip = 0;
    for (std::size_t i = 0; i < restSkin.rest.size(); ++i)
        if (restSkin.rest[i].x > restSkin.rest[tip].x) tip = i;
    skin->mesh.vertices[tip].position += Vec3{0, 3, 0};
    const Vec3 edited = skin->mesh.vertices[tip].position;
    REQUIRE(run(d, "ARMATURE Arm POSE RESET").ok);
    skin = named(d, "Arm skin");
    CHECK(length(skin->mesh.vertices[tip].position - (restSkin.rest[tip] + Vec3{3, 0, 0})) < 1e-3f); // the edit, turned back
    REQUIRE(run(d, "ARMATURE Arm POSE 2 AXIS 0,0,1 ANGLE 90").ok);
    skin = named(d, "Arm skin");
    CHECK(length(skin->mesh.vertices[tip].position - edited) < 1e-3f);
    // UNDO takes the pose back.
    REQUIRE(run(d, "UNDO").ok);
    CHECK(std::fabs(armatureOf(d, "Arm").balls[1].turn[3] - 1) < 1e-6f);
}

TEST_CASE("ARMATURE: add, mirror, insert, move a chain, size, remove, refusals") {
    Document d;
    REQUIRE(run(d, "ARMATURE NEW Figure AT 0,0,100 RADIUS 20").ok);
    REQUIRE(run(d, "ARMATURE Figure ADD AT 0,0,140 RADIUS 14").ok);
    const CommandOutput o = run(d, "ARMATURE Figure ADD TO 1 AT 40,0,100 RADIUS 10 MIRROR X");
    REQUIRE(o.ok);
    CHECK(joined(o).find("and its mirror 4 at (-40, 0, 100)") != std::string::npos);
    REQUIRE(run(d, "ARMATURE Figure ADD TO 3 AT 80,0,100 RADIUS 8 MIRROR X").ok);
    Armature a = armatureOf(d, "Figure");
    REQUIRE(a.balls.size() == 6);
    CHECK(a.balls[4].parent == 2 && a.balls[5].parent == 3); // the mirror's on the mirror
    // Moving a ball and what hangs from it, mirrored.
    REQUIRE(run(d, "ARMATURE Figure MOVE 3 BY 0,0,10 CHAIN MIRROR X").ok);
    a = armatureOf(d, "Figure");
    CHECK(length(a.balls[4].at - Vec3{80, 0, 110}) < 1e-4f);
    CHECK(length(a.balls[5].at - Vec3{-80, 0, 110}) < 1e-4f);
    REQUIRE(run(d, "ARMATURE Figure RADIUS 5 6 MIRROR X").ok);
    a = armatureOf(d, "Figure");
    CHECK(a.balls[4].radius == 6 && a.balls[5].radius == 6);
    REQUIRE(run(d, "ARMATURE Figure INSERT 5").ok);
    a = armatureOf(d, "Figure");
    REQUIRE(a.balls.size() == 7);
    CHECK(a.balls[4].parent == 6 && a.balls[6].parent == 2);
    CHECK(length(a.balls[6].at - Vec3{60, 0, 110}) < 1e-4f);
    REQUIRE(run(d, "ARMATURE Figure POSE 3 TURN 0,30,0 MIRROR X").ok);
    a = armatureOf(d, "Figure");
    CHECK_NEAR(quatToEuler(a.balls[3].turn).y, -30, 1e-3); // the mirror turns the mirrored way
    CHECK(joined(run(d, "ARMATURE Figure LIST")).find("turned (0, 30, 0) deg") != std::string::npos);
    REQUIRE(run(d, "ARMATURE Figure REMOVE 3").ok); // and 5 and 7 on it
    CHECK(armatureOf(d, "Figure").balls.size() == 4);
    CHECK(run(d, "ARMATURE").lines.size() == 1);
    // Refusals.
    CHECK(!run(d, "ARMATURE Figure REMOVE 1").ok);   // the root
    CHECK(!run(d, "ARMATURE Figure ADD TO 9 AT 1,2,3").ok);
    CHECK(!run(d, "ARMATURE Figure ADD AT 1,2,3 MIRROR W").ok);
    CHECK(!run(d, "ARMATURE Figure ADD").ok);
    CHECK(!run(d, "ARMATURE Figure POSE 2").ok);
    CHECK(!run(d, "ARMATURE Nothing LIST").ok);
    CHECK(!run(d, "ARMATURE NEW Figure").ok);
    CHECK(!run(d, "ARMATURE Figure FLY").ok);
}

TEST_CASE("armatures and skins are saved, follow a rename, and work over MCP") {
    Document d;
    REQUIRE(run(d, "ARMATURE NEW Arm AT 0,0,0 RADIUS 10").ok);
    REQUIRE(run(d, "ARMATURE Arm ADD AT 50,0,0 RADIUS 8").ok);
    REQUIRE(run(d, "ARMATURE Arm ADD AT 100,0,0 RADIUS 6").ok);
    REQUIRE(run(d, "ARMATURE Arm SKIN NAME Limb VOXEL 2 BLEND 2 SMOOTH 4").ok);
    REQUIRE(run(d, "ARMATURE Arm POSE 2 TURN 0,0,45").ok);
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_armature_test.sicdm").string();
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Document e;
    REQUIRE(loadProject(e.scene, e.state, path, &err, nullptr));
    std::error_code ec;
    std::filesystem::remove(path, ec);
    Body* limb = named(e, "Limb");
    REQUIRE(limb && limb->skin);
    CHECK(limb->skin->armature == "Arm");
    CHECK(limb->skin->rest.size() == limb->mesh.vertices.size());
    CHECK(named(e, "Arm")->kind == "armature");
    CHECK_NEAR(quatToEuler(armatureOf(e, "Arm").balls[1].turn).z, 45, 1e-3);
    // Posed again after loading: the same as posing it here.
    REQUIRE(run(e, "ARMATURE Arm POSE 2 TURN 0,0,-30").ok);
    REQUIRE(run(d, "ARMATURE Arm POSE 2 TURN 0,0,-30").ok);
    double worst = 0;
    const Body* a = named(d, "Limb");
    limb = named(e, "Limb");
    for (std::size_t i = 0; i < a->mesh.vertices.size(); ++i)
        worst = std::max(worst, static_cast<double>(length(a->mesh.vertices[i].position - limb->mesh.vertices[i].position)));
    CHECK(worst < 1e-3);
    // Renamed, the skin follows it still.
    for (std::size_t i = 0; i < e.scene.bodies().size(); ++i)
        if (e.scene.bodies()[i].name == "Arm") e.scene.renameBody(i, "Rig");
    CHECK(named(e, "Limb")->skin->armature == "Rig");
    REQUIRE(run(e, "ARMATURE Rig POSE RESET").ok);

    Document m;
    McpServer server(m, nullptr);
    const auto tool = [&](const std::string& args) {
        const std::string msg = R"({"jsonrpc":"2.0","id":9,"method":"tools/call","params":{"name":"armature","arguments":)" + args + "}}";
        const auto r = Json::parse(server.handle(msg));
        return r ? (*r)["result"] : Json();
    };
    CHECK(!tool(R"({"operation":"new","armature":"Tail","at":[0,0,0],"radius":12})")["isError"].asBool());
    CHECK(!tool(R"({"operation":"add","armature":"Tail","at":[40,0,0],"radius":9})")["isError"].asBool());
    CHECK(!tool(R"({"operation":"add","armature":"Tail","to":2,"at":[75,0,0],"radius":6})")["isError"].asBool());
    CHECK(!tool(R"({"operation":"skin","armature":"Tail"})")["isError"].asBool());
    CHECK(!tool(R"({"operation":"pose","armature":"Tail","ball":2,"axis":[0,1,0],"angle":30})")["isError"].asBool());
    CHECK(tool(R"({"operation":"list","armature":"Tail"})")["content"][0]["text"].asString().find("turned (0, 30, 0) deg") != std::string::npos);
    CHECK(named(m, "Tail skin") != nullptr);
    CHECK(tool(R"({"operation":"pose","armature":"Tail","ball":9,"angle":30})")["isError"].asBool());
    CHECK(tool(R"({"operation":"fly","armature":"Tail"})")["isError"].asBool());
}

TEST_CASE("armature edits after skinning: the skin follows, what was sculpted on it stays") {
    // Two of the same arm: one with a bump pulled out at its tip.
    auto make = [](Document& d) {
        run(d, "ARMATURE NEW Arm AT 0,0,0 RADIUS 10");
        run(d, "ARMATURE Arm ADD AT 50,0,0 RADIUS 8");
        run(d, "ARMATURE Arm ADD AT 100,0,0 RADIUS 6");
        return run(d, "ARMATURE Arm SKIN").ok;
    };
    Document plain, bumped;
    REQUIRE(make(plain));
    REQUIRE(make(bumped));
    Body* skin = named(bumped, "Arm skin");
    REQUIRE(skin && skin->skin);
    std::size_t tip = 0;
    for (std::size_t i = 0; i < skin->mesh.vertices.size(); ++i)
        if (skin->mesh.vertices[i].position.x > skin->mesh.vertices[tip].position.x) tip = i;
    skin->mesh.vertices[tip].position += Vec3{3, 0, 0}; // pulled out of the tip, 3 mm
    auto both = [&](const std::string& line) { return run(plain, line).ok && run(bumped, line).ok; };
    auto bump = [&]() { // how far the bumped skin's tip vertex is from the plain one's
        const Body *p = named(plain, "Arm skin"), *b = named(bumped, "Arm skin");
        return p && b && p->mesh.vertices.size() == b->mesh.vertices.size() ? length(b->mesh.vertices[tip].position - p->mesh.vertices[tip].position) : -1.0f;
    };
    const std::size_t count = skin->mesh.vertices.size();
    // Moving the tip ball up: the same vertices, the tip end goes with it, the root end stays.
    const std::vector<Vertex> before = named(plain, "Arm skin")->mesh.vertices;
    REQUIRE(both("ARMATURE Arm MOVE 3 BY 0,30,0"));
    const Body* moved = named(plain, "Arm skin");
    REQUIRE(moved->mesh.vertices.size() == count);
    double rootWorst = 0, tipWorst = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const Vec3 was = before[i].position, now = moved->mesh.vertices[i].position;
        if (was.x < 5) rootWorst = std::max(rootWorst, static_cast<double>(length(now - was)));
        if (was.x > 100) tipWorst = std::max(tipWorst, std::fabs(static_cast<double>(length(now - Vec3{100, 30, 0}) - 6))); // on the ball's skin still
    }
    CHECK(rootWorst < 0.05);
    CHECK(tipWorst < 0.6);
    CHECK(openEdgeCount(moved->mesh) == 0);
    CHECK(std::fabs(bump() - 3) < 0.3); // the bump went with it
    // A ball made bigger: its skin grows round it.
    REQUIRE(both("ARMATURE Arm RADIUS 2 12"));
    double around = 0;
    for (const Vertex& v : named(plain, "Arm skin")->mesh.vertices)
        if (std::fabs(v.position.x - 50) < 1.5f) around = std::max(around, static_cast<double>(std::fabs(v.position.z)));
    CHECK(std::fabs(around - 12) < 0.8);
    CHECK(std::fabs(bump() - 3) < 0.3);
    // A ball put halfway along a link: the skin stays put, and the new ball bends what is past it.
    REQUIRE(both("ARMATURE Arm INSERT 3"));
    REQUIRE(named(plain, "Arm skin")->mesh.vertices.size() == count);
    const std::vector<Vertex> straight = named(plain, "Arm skin")->mesh.vertices;
    REQUIRE(both("ARMATURE Arm POSE 4 AXIS 0,0,1 ANGLE 30"));
    double nearBall2 = 0, pastMid = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const double d = length(named(plain, "Arm skin")->mesh.vertices[i].position - straight[i].position);
        if (straight[i].position.x < 52) nearBall2 = std::max(nearBall2, d);
        if (straight[i].position.x > 95) pastMid = std::max(pastMid, d);
    }
    CHECK(nearBall2 < 0.05);
    CHECK(pastMid > 5);
    REQUIRE(both("ARMATURE Arm POSE RESET"));
    // A ball added: skinned again (more of it), still bound, the bump carried over.
    REQUIRE(both("ARMATURE Arm ADD TO 2 AT 50,0,50 RADIUS 5"));
    const Body *ps = named(plain, "Arm skin"), *bs = named(bumped, "Arm skin");
    REQUIRE(ps && bs && ps->skin && bs->skin);
    CHECK(ps->mesh.vertices.size() > count);
    double up = 0;
    for (const Vertex& v : ps->mesh.vertices) up = std::max(up, static_cast<double>(v.position.z));
    CHECK(std::fabs(up - 55) < 0.8); // over the new ball
    REQUIRE(ps->mesh.vertices.size() == bs->mesh.vertices.size());
    double most = 0, elsewhere = 0;
    for (std::size_t i = 0; i < ps->mesh.vertices.size(); ++i) {
        const double d = length(bs->mesh.vertices[i].position - ps->mesh.vertices[i].position);
        most = std::max(most, d);
        if (ps->mesh.vertices[i].position.x < 80) elsewhere = std::max(elsewhere, d);
    }
    CHECK(most > 2.0);      // the bump is there
    CHECK(elsewhere < 0.05); // and nowhere else
    CHECK(openEdgeCount(ps->mesh) == 0);
    // Taken away again: still bound (it used to be unbound).
    REQUIRE(both("ARMATURE Arm REMOVE 5"));
    CHECK(named(plain, "Arm skin")->skin != nullptr);
    CHECK(named(plain, "Arm skin")->mesh.vertices.size() < ps->mesh.vertices.size() + 1);
    // Saved and opened again: its unsculpted shape and options come back, so a later edit
    // still keeps the bump.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_tests" / "refit.sicdm").string();
    std::string why;
    REQUIRE(saveProject(bumped.scene, bumped.state, path, &why));
    Document again;
    REQUIRE(loadProject(again.scene, again.state, path, &why));
    std::filesystem::remove(path);
    const Body* loaded = named(again, "Arm skin");
    REQUIRE(loaded && loaded->skin);
    CHECK(loaded->skin->base.size() == loaded->skin->rest.size());
    CHECK(std::fabs(loaded->skin->options.blend - named(bumped, "Arm skin")->skin->options.blend) < 1e-6f);
    // UNDO puts the skin back as it was before the last edit.
    const std::size_t afterRemove = named(plain, "Arm skin")->mesh.vertices.size();
    REQUIRE(run(plain, "UNDO").ok);
    CHECK(named(plain, "Arm skin")->mesh.vertices.size() != afterRemove);
}

TEST_CASE("armature: a remeshed skin is bound again with the weights it had, not by distance") {
    Document d;
    REQUIRE(run(d, "ARMATURE NEW Arm AT 0,0,0 RADIUS 10").ok);
    REQUIRE(run(d, "ARMATURE Arm ADD AT 50,0,0 RADIUS 8").ok);
    REQUIRE(run(d, "ARMATURE Arm ADD AT 100,0,0 RADIUS 6").ok);
    REQUIRE(run(d, "ARMATURE Arm SKIN").ok);
    Body* skin = named(d, "Arm skin");
    REQUIRE(skin && skin->skin);
    // Weighted by hand as distance never would: all of it on the first link (0 to 1), so a
    // turn at ball 2 moves none of it. (A blend of the two links where x < 0, to see it carried.)
    auto w = std::make_shared<SkinData>(*skin->skin);
    for (std::size_t i = 0; i < w->rest.size(); ++i) {
        if (w->rest[i].x < 0) w->bone[i] = {1, 2, -1, -1}, w->weight[i] = {0.75f, 0.25f, 0, 0};
        else w->bone[i] = {1, -1, -1, -1}, w->weight[i] = {1, 0, 0, 0};
    }
    skin->skin = w;
    REQUIRE(run(d, "SUBDIVIDE \"Arm skin\"").ok);
    skin = named(d, "Arm skin");
    REQUIRE(skin->mesh.vertices.size() != skin->skin->rest.size());
    const std::vector<Vertex> before = skin->mesh.vertices;
    const CommandOutput posed = run(d, "ARMATURE Arm POSE 2 AXIS 0,0,1 ANGLE 90");
    REQUIRE(posed.ok);
    CHECK(joined(posed).find("bound to Arm again") != std::string::npos);
    skin = named(d, "Arm skin");
    REQUIRE(skin->skin->rest.size() == skin->mesh.vertices.size());
    double moved = 0;
    for (std::size_t i = 0; i < before.size(); ++i)
        if (before[i].position.x > 2) moved = std::max(moved, static_cast<double>(length(skin->mesh.vertices[i].position - before[i].position)));
    CHECK(moved < 1e-3); // by distance the tip would swing round 90 deg (the blend behind does turn a little)
    std::size_t off = 0, blended = 0, behind = 0;
    for (std::size_t i = 0; i < before.size(); ++i) {
        const auto& bone = skin->skin->bone[i];
        const auto& weight = skin->skin->weight[i];
        float on1 = 0, total = 0;
        for (std::size_t q = 0; q < 4; ++q) {
            if (bone[q] < 0) continue;
            total += weight[q];
            if (bone[q] == 1) on1 += weight[q];
        }
        CHECK(std::fabs(total - 1) < 1e-4f);
        if (before[i].position.x > 2 && on1 < 0.999f) ++off;
        if (before[i].position.x < -2) {
            ++behind;
            if (std::fabs(on1 - 0.75f) < 1e-3f) ++blended;
        }
    }
    CHECK(off == 0);
    CHECK(behind > 10);
    CHECK(blended == behind);
}

TEST_CASE("armature: skinned again keeps polygroups, UVs and paint; a remeshed skin is bound again") {
    Document d;
    REQUIRE(run(d, "ARMATURE NEW Arm AT 0,0,0 RADIUS 10").ok);
    REQUIRE(run(d, "ARMATURE Arm ADD AT 50,0,0 RADIUS 8").ok);
    REQUIRE(run(d, "ARMATURE Arm ADD AT 100,0,0 RADIUS 6").ok);
    REQUIRE(run(d, "ARMATURE Arm SKIN").ok);
    Body* skin = named(d, "Arm skin");
    REQUIRE(skin && skin->skin);
    // Polygroups (two halves), UVs (a plain projection: u = x / 100, v = z / 100) and paint.
    MeshData& m = skin->mesh;
    m.groups.assign(m.indices.size() / 3, 0);
    m.uvs.assign(m.indices.size(), Vec2{0, 0});
    for (std::size_t t = 0; t < m.groups.size(); ++t) {
        Vec3 c{0, 0, 0};
        for (int k = 0; k < 3; ++k) {
            const Vec3 p = m.vertices[m.indices[3 * t + static_cast<std::size_t>(k)]].position;
            c += p * (1.0f / 3.0f);
            m.uvs[3 * t + static_cast<std::size_t>(k)] = {p.x / 100, p.z / 100};
        }
        m.groups[t] = c.x > 50 ? 2u : 1u;
    }
    for (Vertex& v : m.vertices) v.color = 0xff2040c0u;
    // A ball added: skinned again (more of it), all three carried over.
    REQUIRE(run(d, "ARMATURE Arm ADD TO 2 AT 50,0,40 RADIUS 6").ok);
    skin = named(d, "Arm skin");
    const MeshData& n = skin->mesh;
    REQUIRE(n.hasGroups());
    REQUIRE(n.hasUvs());
    std::size_t wrong = 0, far = 0, uvOff = 0;
    for (std::size_t t = 0; t < n.indices.size() / 3; ++t) {
        Vec3 c{0, 0, 0};
        for (int k = 0; k < 3; ++k) c += n.vertices[n.indices[3 * t + static_cast<std::size_t>(k)]].position * (1.0f / 3.0f);
        if (c.z > 15) continue; // the new ball's own
        if (std::fabs(c.x - 50) > 5) {
            ++far;
            wrong += n.groups[t] != (c.x > 50 ? 2u : 1u) ? 1 : 0;
        }
        for (int k = 0; k < 3; ++k) { // where nothing changed the projection comes back exactly
            const Vec3 p = n.vertices[n.indices[3 * t + static_cast<std::size_t>(k)]].position;
            const Vec2 uv = n.uvs[3 * t + static_cast<std::size_t>(k)];
            if (std::fabs(c.x - 50) > 15 && (std::fabs(uv.x - p.x / 100) > 0.01f || std::fabs(uv.y - p.z / 100) > 0.01f)) ++uvOff;
        }
    }
    CHECK(far > 100);
    CHECK(wrong == 0);
    CHECK(uvOff == 0);
    std::size_t unpainted = 0;
    for (const Vertex& v : n.vertices) unpainted += v.color != 0xff2040c0u ? 1 : 0;
    CHECK(unpainted == 0); // the new ball's skin takes the paint round it
    // Remeshed (subdivided: other vertices): bound again, and it follows a pose.
    REQUIRE(run(d, "SUBDIVIDE \"Arm skin\"").ok);
    skin = named(d, "Arm skin");
    REQUIRE(skin->mesh.vertices.size() != skin->skin->rest.size());
    const std::vector<Vertex> before = skin->mesh.vertices;
    const CommandOutput posed = run(d, "ARMATURE Arm POSE 2 AXIS 0,0,1 ANGLE 90");
    REQUIRE(posed.ok);
    CHECK(joined(posed).find("bound to Arm again") != std::string::npos);
    skin = named(d, "Arm skin");
    REQUIRE(skin->skin->rest.size() == skin->mesh.vertices.size());
    double tipMoved = 0, rootMoved = 0;
    for (std::size_t i = 0; i < before.size(); ++i) {
        const double moved = length(skin->mesh.vertices[i].position - before[i].position);
        if (before[i].position.x > 95) tipMoved = std::max(tipMoved, moved);
        if (before[i].position.x < 5) rootMoved = std::max(rootMoved, moved);
    }
    CHECK(tipMoved > 40); // turned 90 deg about ball 2
    CHECK(rootMoved < 0.5);
    CHECK(openEdgeCount(skin->mesh) == 0);
    // And a later edit keeps its shape: back to rest, a ball moved, still closed and bound.
    REQUIRE(run(d, "ARMATURE Arm POSE RESET").ok);
    REQUIRE(run(d, "ARMATURE Arm MOVE 3 BY 0,0,10").ok);
    skin = named(d, "Arm skin");
    CHECK(skin->skin != nullptr);
    CHECK(openEdgeCount(skin->mesh) == 0);
}

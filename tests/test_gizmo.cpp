#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Gizmo.hpp"
#include "core/Scene.hpp"

#include <cmath>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

// Looking straight down -Z: 10 pixels a millimetre, the origin at (500, 500), +Y up the screen.
const Vec3 kDown{0, 0, -1};
std::optional<Vec2> topView(const Vec3& p) { return Vec2{500 + p.x * 10, 500 - p.y * 10}; }
GizmoRay rayAt(float sx, float sy) { return {{(sx - 500) / 10, (500 - sy) / 10, 100}, kDown, {sx, sy}}; }

Body* named(Document& d, const std::string& n) {
    for (Body& b : d.scene.bodies())
        if (b.name == n) return &b;
    return nullptr;
}

bool near(const Vec3& a, const Vec3& b, float tol) { return length(a - b) <= tol; }

} // namespace

TEST_CASE("gizmo: the handle under the mouse, in each mode") {
    const Vec3 P{0, 0, 0};
    const float size = 10; // 100 pixels
    // Move: the X arrow, the Y arrow, the square of the XY plane, the centre.
    CHECK(gizmoPick(GizmoMode::Move, P, size, kDown, topView, {590, 500}) == (GizmoHandle{GizmoHandle::Axis, 0}));
    CHECK(gizmoPick(GizmoMode::Move, P, size, kDown, topView, {500, 420}) == (GizmoHandle{GizmoHandle::Axis, 1}));
    CHECK(gizmoPick(GizmoMode::Move, P, size, kDown, topView, {530, 470}) == (GizmoHandle{GizmoHandle::Plane, 2}));
    CHECK(gizmoPick(GizmoMode::Move, P, size, kDown, topView, {500, 500}).kind == GizmoHandle::Center);
    CHECK(gizmoPick(GizmoMode::Move, P, size, kDown, topView, {700, 700}).kind == GizmoHandle::None);
    // Rotate: seen from above, the Z ring is the circle; the view ring is outside it.
    CHECK(gizmoPick(GizmoMode::Rotate, P, size, kDown, topView, {500 + 70.7f, 500 - 70.7f}) == (GizmoHandle{GizmoHandle::Ring, 2}));
    CHECK(gizmoPick(GizmoMode::Rotate, P, size, kDown, topView, {500 + 118, 500}).kind == GizmoHandle::ViewRing);
    // Scale: the cube on the X axis, the centre.
    CHECK(gizmoPick(GizmoMode::Scale, P, size, kDown, topView, {600, 500}) == (GizmoHandle{GizmoHandle::Axis, 0}));
    CHECK(gizmoPick(GizmoMode::Scale, P, size, kDown, topView, {502, 501}).kind == GizmoHandle::Center);
    CHECK(!gizmoStrokes(GizmoMode::Rotate, P, size, kDown, topView).empty());
}

TEST_CASE("gizmo drags: along an axis, in a plane, turning, scaling, with snapping") {
    const Vec3 P{0, 0, 0};
    // Along X: the mouse 15 mm right (and a little up: only X counts).
    GizmoDelta d = gizmoDrag(GizmoMode::Move, {GizmoHandle::Axis, 0}, P, kDown, topView, rayAt(600, 500), rayAt(750, 480));
    CHECK(near(d.move, {15, 0, 0}, 1e-3f));
    d = gizmoDrag(GizmoMode::Move, {GizmoHandle::Axis, 0}, P, kDown, topView, rayAt(600, 500), rayAt(750, 480), 10);
    CHECK(near(d.move, {20, 0, 0}, 1e-3f)); // snapped to 10 mm
    // In the XY plane.
    d = gizmoDrag(GizmoMode::Move, {GizmoHandle::Plane, 2}, P, kDown, topView, rayAt(530, 470), rayAt(580, 420));
    CHECK(near(d.move, {5, 5, 0}, 1e-3f));
    // Turning about Z, which points at the viewer: a quarter turn anticlockwise on screen is +90.
    d = gizmoDrag(GizmoMode::Rotate, {GizmoHandle::Ring, 2}, P, kDown, topView, rayAt(600, 500), rayAt(500, 400));
    CHECK(near(d.axis, {0, 0, 1}, 1e-6f));
    CHECK(std::fabs(d.angle - 90) < 1e-3f);
    d = gizmoDrag(GizmoMode::Rotate, {GizmoHandle::Ring, 2}, P, kDown, topView, rayAt(600, 500), rayAt(590, 450), 15);
    CHECK(std::fabs(d.angle - 30) < 1e-3f); // 26.6 deg snapped to 15s
    // The view ring turns about the line of sight, towards the viewer.
    d = gizmoDrag(GizmoMode::Rotate, {GizmoHandle::ViewRing, 0}, P, kDown, topView, rayAt(600, 500), rayAt(500, 400));
    CHECK(near(d.axis, {0, 0, 1}, 1e-6f));
    // Scaling along X: from 10 mm out to 25 mm out is x2.5, only along X.
    d = gizmoDrag(GizmoMode::Scale, {GizmoHandle::Axis, 0}, P, kDown, topView, rayAt(600, 500), rayAt(750, 500));
    CHECK(!d.uniform);
    CHECK(near(d.scale, {2.5f, 1, 1}, 1e-4f));
    // The centre scales evenly by how far from the pivot the mouse goes on screen.
    d = gizmoDrag(GizmoMode::Scale, {GizmoHandle::Center, 0}, P, kDown, topView, rayAt(550, 500), rayAt(600, 500), 0.25f);
    CHECK(d.uniform && std::fabs(d.scale.x - 2) < 1e-4f && d.scale.z == d.scale.x);
}

TEST_CASE("gizmo commands do what the drag showed, and are undone") {
    Document d;
    REQUIRE(run(d, "BOX 20 10 10 @10,0,0").ok);
    Body* b = named(d, "Block 1");
    REQUIRE(b != nullptr);
    const Vec3 pivot = b->bounds().center();
    // The preview of a turn about the pivot, and the command the drag ends in, agree.
    Body preview = *b;
    REQUIRE(turnBody(preview, {0, 0, 1}, 90, pivot));
    GizmoDelta turn;
    turn.axis = {0, 0, 1}, turn.angle = 90;
    const std::string line = gizmoCommand(GizmoMode::Rotate, turn, "Block 1", pivot);
    CHECK(line == "ROTATE \"Block 1\" BY 90 AXIS 0,0,1 ABOUT 10mm,0mm,5mm");
    REQUIRE(run(d, line).ok);
    b = named(d, "Block 1");
    CHECK(near(b->bounds().min, preview.bounds().min, 1e-4f) && near(b->bounds().max, preview.bounds().max, 1e-4f));
    CHECK(near(b->bounds().center(), pivot, 1e-4f)); // turned in place
    CHECK(std::fabs((b->bounds().max.y - b->bounds().min.y) - 20) < 1e-3f);
    CHECK(b->kind == "box"); // still the parametric box, turned
    REQUIRE(run(d, "UNDO").ok);
    CHECK(std::fabs(named(d, "Block 1")->bounds().max.x - 20) < 1e-4f);
    // Scaling about the pivot: evenly (it stays a box), and along one axis (a mesh now).
    GizmoDelta k;
    k.scale = {2, 2, 2};
    REQUIRE(run(d, gizmoCommand(GizmoMode::Scale, k, "Block 1", pivot)).ok);
    b = named(d, "Block 1");
    CHECK(near(b->bounds().center(), pivot, 1e-4f));
    CHECK(std::fabs((b->bounds().max.x - b->bounds().min.x) - 40) < 1e-3f);
    CHECK(b->kind == "box");
    k.scale = {1, 3, 1}, k.uniform = false;
    REQUIRE(run(d, gizmoCommand(GizmoMode::Scale, k, "Block 1", pivot)).ok);
    b = named(d, "Block 1");
    CHECK(std::fabs((b->bounds().max.y - b->bounds().min.y) - 60) < 1e-3f);
    CHECK(near(b->bounds().center(), pivot, 1e-3f));
    // Moving.
    GizmoDelta m;
    m.move = {5, -2, 1};
    CHECK(gizmoCommand(GizmoMode::Move, m, "Block 1", pivot) == "MOVE 5mm,-2mm,1mm \"Block 1\"");
    REQUIRE(run(d, gizmoCommand(GizmoMode::Move, m, "Block 1", pivot)).ok);
    CHECK(near(named(d, "Block 1")->bounds().center(), pivot + Vec3{5, -2, 1}, 1e-3f));
    // Nothing done, nothing to run.
    CHECK(gizmoCommand(GizmoMode::Rotate, GizmoDelta{}, "Block 1", pivot).empty());
    CHECK(gizmoCommand(GizmoMode::Scale, GizmoDelta{}, "Block 1", pivot).empty());
    // Refusals.
    CHECK(!run(d, "ROTATE \"Block 1\" BY").ok);
    CHECK(!run(d, "ROTATE \"Block 1\" BY 30 AXIS W").ok);
    CHECK(!run(d, "SCALE \"Block 1\" 2 ABOUT").ok);
    CHECK(!run(d, "SCALE \"Block 1\" 0 ABOUT 0,0,0").ok);
    CHECK(!run(d, "GIZMO ROTATE").ok); // needs the app's window
    CHECK(!run(d, "GIZMO SPIN").ok);
}

TEST_CASE("gizmo: a turn goes on past half a turn, frame by frame, any number of turns") {
    const Vec3 P{0, 0, 0};
    // The mouse goes round the pivot anticlockwise on screen, one and a half times, a frame
    // every 10 degrees.
    const GizmoRay start = rayAt(600, 500);
    GizmoDelta d;
    for (int k = 1; k <= 54; ++k) {
        const float a = static_cast<float>(k) * 10.0f * 3.14159265f / 180.0f;
        d = gizmoDrag(GizmoMode::Rotate, {GizmoHandle::Ring, 2}, P, kDown, topView, start,
                      rayAt(500 + 100 * std::cos(a), 500 - 100 * std::sin(a)), 0, &d);
        if (k == 18) CHECK(std::fabs(std::fabs(d.angle) - 180) < 1e-2f);
        if (k == 27) CHECK(std::fabs(d.angle - 270) < 1e-2f); // not -90
    }
    CHECK(std::fabs(d.angle - 540) < 1e-2f);
    // And back the other way, through zero to -90.
    for (int k = 53; k >= -9; --k) {
        const float a = static_cast<float>(k) * 10.0f * 3.14159265f / 180.0f;
        d = gizmoDrag(GizmoMode::Rotate, {GizmoHandle::Ring, 2}, P, kDown, topView, start,
                      rayAt(500 + 100 * std::cos(a), 500 - 100 * std::sin(a)), 15, &d);
    }
    CHECK(std::fabs(d.angle + 90) < 1e-2f);
    // Without the last frame (one jump), it is the short way round, as before.
    d = gizmoDrag(GizmoMode::Rotate, {GizmoHandle::Ring, 2}, P, kDown, topView, start, rayAt(500, 600));
    CHECK(std::fabs(d.angle + 90) < 1e-2f);
    // The command turns the body that far: 450 degrees ends where 90 does.
    Document doc;
    REQUIRE(run(doc, "BOX 20 10 10").ok);
    REQUIRE(run(doc, "ROTATE \"Block 1\" BY 450 AXIS Z").ok);
    CHECK(std::fabs(named(doc, "Block 1")->rotation.z - 90) < 1e-3f);
}

TEST_CASE("gizmo: a move snaps to another body's corner, edge or face middle near the mouse") {
    Document d;
    REQUIRE(run(d, "BOX 10 10 10").ok);          // Block 1: x -5..5, y -5..5, z 0..10
    REQUIRE(run(d, "BOX 10 10 10 @30,0,0").ok);  // Block 2: x 25..35
    const Body& a = d.scene.bodies()[0];
    const Body& b = d.scene.bodies()[1];
    const std::vector<Vec3> from = snapPoints(a), to = snapPoints(b);
    CHECK(from.size() >= 15); // box corners, face middles, centre, and the edges' ends and middles
    // Dragged 19 mm along X: its right side is 1 mm (10 pixels) short of Block 2's left side.
    auto hit = gizmoSnap(from, to, {19, 0, 0}, topView, 12.0f);
    REQUIRE(hit.has_value());
    CHECK(near(hit->move, {20, 0, 0}, 1e-4f)); // face to face, not up or down
    CHECK(std::fabs(hit->to.x - 25) < 1e-4f);
    // Too far (3 mm: 30 pixels): no snap.
    CHECK(!gizmoSnap(from, to, {17, 0, 0}, topView, 12.0f));
    // Along the X arrow only: the Y part of the gap is not taken.
    hit = gizmoSnap(from, to, {19, 0.6f, 0}, topView, 12.0f, {1, 0, 0});
    REQUIRE(hit.has_value());
    CHECK(near(hit->move, {20, 0, 0}, 1e-4f));
    // In the XY plane only: no Z.
    hit = gizmoSnap(from, to, {19, 0.5f, 0}, topView, 12.0f, {}, {0, 0, 1});
    REQUIRE(hit.has_value());
    CHECK(std::fabs(hit->move.z) < 1e-6f);
    CHECK(std::fabs(hit->move.x - 20) < 1e-4f);
    // A sculpted body (no edges drawn) still has its box's points; the caps hold.
    CHECK(snapPoints(b, 10).size() <= 16);
    CHECK(gizmoSnap({}, to, {}, topView) == std::nullopt);
}

TEST_CASE("gizmo: a free move snaps in full 3D to the points allowed, the next nearest when one isn't") {
    Document d;
    REQUIRE(run(d, "BOX 10 10 10").ok);          // z 0..10
    REQUIRE(run(d, "BOX 10 10 10 @30,0,30").ok); // z 30..40, seen from above
    const std::vector<Vec3> from = snapPoints(d.scene.bodies()[0]), to = snapPoints(d.scene.bodies()[1]);
    // From the top, Block 1's corners line up with Block 2's after 19 mm along X; in full 3D
    // the nearest pair in the model is Block 1's top corner to Block 2's bottom one: up 20.
    auto hit = gizmoSnap(from, to, {19, 0, 0}, topView, 12.0f);
    REQUIRE(hit.has_value());
    CHECK(near(hit->move, {20, 0, 20}, 1e-4f));
    // Only Block 2's top corners allowed (its bottom hidden from this view): onto those.
    hit = gizmoSnap(from, to, {19, 0, 0}, topView, 12.0f, {}, {}, [&](std::size_t i) { return to[i].z > 39; });
    REQUIRE(hit.has_value());
    CHECK(near(hit->move, {20, 0, 30}, 1e-4f));
    // None allowed: no snap.
    CHECK(!gizmoSnap(from, to, {19, 0, 0}, topView, 12.0f, {}, {}, [](std::size_t) { return false; }));
}

#pragma once

// The move / rotate / scale gizmo: its handles round a pivot, which one is under the
// mouse, what a drag on it does, and the body transforms the drag ends in (also the
// commands' own: ROTATE ... BY ... ABOUT, SCALE ... ABOUT, MOVE).
//
// Everything is in model space (mm) except the mouse, which is in pixels: the gizmo keeps
// one size on screen, `size` mm long at the pivot.

#include "core/Math.hpp"

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

struct Body;

enum class GizmoMode { Move, Rotate, Scale };

struct GizmoHandle {
    enum Kind { None, Axis, Plane, Center, Ring, ViewRing } kind = None;
    int axis = 0; // 0 X, 1 Y, 2 Z (Plane: the axis square to the plane)
    bool operator==(const GizmoHandle& o) const { return kind == o.kind && (kind == None || kind == Center || kind == ViewRing || axis == o.axis); }
};

// Model space to screen pixels (nothing when behind the camera).
using GizmoProject = std::function<std::optional<Vec2>(const Vec3&)>;

// The handle within `reach` pixels of the mouse (the nearest), or None.
GizmoHandle gizmoPick(GizmoMode mode, const Vec3& pivot, float size, const Vec3& viewDir, const GizmoProject& project, const Vec2& mouse, float reach = 8.0f);

// The screen polylines that draw each handle (for drawing and picking alike).
struct GizmoStroke {
    GizmoHandle handle;
    std::vector<Vec2> points;
    bool closed = false;
    bool fill = false; // a filled shape (a plane square, a scale cube, the centre)
};
std::vector<GizmoStroke> gizmoStrokes(GizmoMode mode, const Vec3& pivot, float size, const Vec3& viewDir, const GizmoProject& project);

// What a drag does so far.
struct GizmoDelta {
    Vec3 move{0, 0, 0};       // Move: mm
    Vec3 axis{0, 0, 1};       // Rotate: about this, through the pivot
    float angle = 0;          // degrees (snapped)
    float rawAngle = 0;       // degrees as the mouse went, unsnapped and wound on past a half turn
    Vec3 scale{1, 1, 1};      // Scale: factors along X, Y, Z (all equal: uniform)
    bool uniform = true;
};

// A drag from `start` to `now` (rays from the camera through the mouse, and the mouse in
// pixels), on `handle`. `step` > 0 snaps: mm, degrees or a factor. `previous`: the drag's
// last result, so a turn goes on past 180 degrees (any number of turns) instead of flipping.
struct GizmoRay {
    Vec3 origin, dir;
    Vec2 mouse;
};
GizmoDelta gizmoDrag(GizmoMode mode, const GizmoHandle& handle, const Vec3& pivot, const Vec3& viewDir, const GizmoProject& project,
                     const GizmoRay& start, const GizmoRay& now, float step = 0, const GizmoDelta* previous = nullptr);

// The command that does it to body `name`, or to several bodies together (empty: nothing to do).
std::string gizmoCommand(GizmoMode mode, const GizmoDelta& d, const std::string& name, const Vec3& pivot);
std::string gizmoCommand(GizmoMode mode, const GizmoDelta& d, const std::vector<std::string>& names, const Vec3& pivot);

// Snapping a move to other bodies: their corners, the middles of their faces and edges, the
// ends of their edges. snapPoints gives a body's (model space; at most `cap`, the bounding
// box's first), gizmoSnap the closest pair on screen within `reach` pixels: a point of what
// moves (`from`, moved by `move`) and a point to put it on (`to`). `axis` (when not zero)
// keeps the move along that line, `plane` (when not zero) in the plane square to it; the
// result is the whole move that puts `from` there (or nothing: no pair close enough).
std::vector<Vec3> snapPoints(const Body& b, std::size_t cap = 20000);
struct GizmoSnapHit {
    Vec3 move{}; // the move with the snap
    Vec3 from{}, to{};
};
std::optional<GizmoSnapHit> gizmoSnap(const std::vector<Vec3>& moving, const std::vector<Vec3>& targets, const Vec3& move,
                                      const GizmoProject& project, float reach = 12.0f, const Vec3& axis = {}, const Vec3& plane = {});

// The transforms themselves (the commands and the live preview use these).
// Turns a body by `degrees` about `axis` through `about` (model space): its orientation and
// where its origin is. False (why) for a body that can't turn on its own.
bool turnBody(Body& b, const Vec3& axis, float degrees, const Vec3& about, std::string* why = nullptr);
// Scales a body by `k` (uniform: all three equal, the body keeps its kind) about `about`.
bool stretchBody(Body& b, const Vec3& k, const Vec3& about, std::string* why = nullptr);

} // namespace sicdm

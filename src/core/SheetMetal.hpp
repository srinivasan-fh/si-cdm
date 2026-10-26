#pragma once

// Sheet metal: a part made of one sheet, thickness t, bent along straight lines.
//
// The part is kept as its flat pattern: a base plate (a closed outline) and flanges, each a
// strip on a free edge of the base or of another flange's far edge, bent through an angle
// with an inside radius. The 3D part is that flat plate folded: in a bend the sheet follows
// the arc (its neutral layer, at K t from the inside, keeping its length), beyond it the
// flange is carried round rigidly. So the folded part and its flat pattern always agree, and
// the flat length of a bend is the bend allowance BA = angle (r + K t).
//
// Body kind "sheet"; its params: [t, r, K, ox, oy, oz, ux, uy, uz, vx, vy, vz (the base's
// plane), n, x0, y0 ... (the base outline), m, then per flange: parent (-1: the base),
// edge, length, angle (deg), down (0/1), radius (-1: the part's)].

#include "core/Geometry.hpp"
#include "core/Sketch.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

struct SheetFlange {
    int parent = -1;     // -1: the base; else the flange whose far edge it's on
    int edge = 0;        // the base outline's edge (from corner `edge` to the next)
    float length = 10;   // the flat beyond the bend
    float angle = 90;    // degrees, 0 to 180
    bool down = false;   // bent the other way (towards -normal)
    float radius = -1;   // inside bend radius (-1: the part's)
};

struct SheetPart {
    float thickness = 1, radius = 1, k = 0.44f;
    PlaneFrame frame = planeFrame(SketchPlane::Top); // the base's plane
    std::vector<Vec2> base;                           // counter-clockwise
    std::vector<SheetFlange> flanges;
};

std::vector<float> sheetParams(const SheetPart& part);
bool sheetFromParams(const std::vector<float>& params, SheetPart& out);

// The bend allowance: the flat length of a bend.
float bendAllowance(float angleDeg, float radius, float k, float thickness);

// The folded part (or with `flat`, its flat pattern, on the base's plane). Empty (why) when a
// flange's edge is gone or the values make no sense.
MeshData buildSheet(const SheetPart& part, bool flat, std::string* why = nullptr);
MeshData buildSheetParams(const std::vector<float>& params);

// The edges a new flange can go on: the base's edges without one, and flanges' far edges
// without one. Each with its ends in the flat (plane coordinates) and folded (world).
struct SheetEdge {
    int parent = -1, edge = 0;
    Vec2 a, b;
    Vec3 a3, b3;
    float length = 0;
};
std::vector<SheetEdge> sheetFreeEdges(const SheetPart& part);

// The flat pattern's outline (closed loops, plane coordinates) and its bend lines (the
// middle of each bend zone), for drawings and DXF.
void sheetFlatOutline(const SheetPart& part, std::vector<std::vector<Vec2>>& outline, std::vector<std::pair<Vec2, Vec2>>& bendLines);

} // namespace sicdm

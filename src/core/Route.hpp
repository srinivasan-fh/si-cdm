#pragma once

// Routes: pipes, tubes, bars, cables and hoses along 3D paths (PATH3D curves), and the
// bend table that bends them.
//
//   Sizes     PIPE DN6..DN150 (EN 10255 medium series), PIPE NPS 1/8..8 SCH 40 | 80
//             (ASME B36.10M), PIPE / TUBE / HOSE od x wall, ROD d, CABLE d
//   Bends     a 3D polyline's corners bend at a centre-line radius: the path's own, or
//             one given, else 1.5 x OD for pipe, 2 x OD for tube and rod and 6 x OD for
//             cable and hose. A 3D spline is followed as it curves (cables and hoses).
//   Solids    exact: cylinders along the straights, tori round the bends, flat ends
//   Bend table  per bend: the straight fed before it, the turn of the tube since the
//             last bend (rotation) and the bend angle (LRA, as tube benders take it),
//             then the last straight; the developed (cut) length and the mass

#include "core/Brep.hpp"
#include "core/Scene.hpp"

#include <optional>
#include <string>
#include <vector>

namespace sicdm {

struct RouteSpec {
    std::string kind;        // "pipe", "tube", "rod", "cable", "hose"
    std::string spec;        // canonical, reads back: "PIPE DN20", "PIPE NPS 3/4 SCH 80", "TUBE 12x1"
    std::string designation; // "Pipe DN20 (26.9 x 2.6)"
    float od = 0, wall = 0;  // mm; wall 0: solid
    bool flexible = false;   // cable, hose
    float defaultBend() const noexcept;
};

// A size from words such as "PIPE DN20", "pipe nps 3/4 sch 80", "TUBE 12x1", "ROD 10",
// "CABLE 8", "HOSE 19x3.5". Nothing (with why) when it isn't one.
std::optional<RouteSpec> routeSpec(const std::string& text, std::string* why = nullptr);
// The standard sizes of a kind ("pipe": DN and NPS sizes), for menus.
std::vector<std::string> routeSizes(const std::string& kind);

// The centre line a route follows, finely (`chord` mm), first to last (a closed one's
// first point repeated).
std::vector<Vec3> routeCentreline(const Body::Route& r, float chord = 0.05f);

// A route's solid (in the frame of its points) and its exact surfaces. False (with why)
// when its bends don't fit or are too tight for its size.
bool buildRoute(const Body::Route& r, MeshData& mesh, std::vector<BSurface>& surfaces, std::string* why = nullptr);

struct RouteBend {
    Vec3 corner;          // the path's point it bends round
    double feed = 0;      // the straight before it (mm)
    double rotation = 0;  // the tube turned about its axis since the last bend (degrees, right hand about the feed)
    double angle = 0;     // the bend (degrees)
    double radius = 0;    // centre line (mm)
    double arc = 0;       // the bend's length along the centre line (mm)
};
struct RouteReport {
    std::vector<RouteBend> bends; // a polyline route's, in order
    double lastFeed = 0;          // the straight after the last bend (an open route's)
    double length = 0;            // developed: straights and arcs (the length to cut)
    double area = 0;              // the section (mm^2)
    double mass = 0;              // kg
    std::string why;              // when the bends don't fit
};
RouteReport routeReport(const Body::Route& r);

} // namespace sicdm

#pragma once

// Drawings: a sheet of 2D views of the model, the way a shop reads a part.
//
//   Views       front, back, top, bottom, left, right and isometric projections, laid out
//               in first-angle (ISO) or third-angle (ASME) order and lined up with each
//               other; section views cut through a view, hatched where the cut meets
//               material
//   Lines       the model's edges (exact faces' boundaries, tangent edges left out) and
//               silhouettes, visible ones thick and hidden ones dashed; centre marks and
//               centre lines on holes and bosses
//   Dimensions  horizontal, vertical and aligned lengths, diameters and radii, tied to
//               model points so they follow the model; automatic overall sizes and hole
//               diameters
//   Tolerances  on dimensions: +/-, deviations, limits, basic (boxed) and ISO 286 fits;
//               geometric tolerances (GD&T, ISO 1101 / ASME Y14.5): feature control
//               frames with their symbol, zone, material condition and datum
//               references, datum feature symbols, surface finish marks (ISO 1302) and
//               the general tolerance (ISO 2768) in the title block
//   Callouts    hole callouts on a leader (ASME Y14.5 style): drill diameter, depth or THRU,
//               counterbore, countersink and thread, "4X" for holes alike
//   Flat views  a sheet metal part's flat pattern with its bend lines and bend notes
//   Sheet       ISO and ANSI sizes, a border, a title block and the projection symbol
//   Export      PDF, SVG and DXF (layers and line types for visible, hidden, centre,
//               dimension and hatch lines)
//
// Sheet coordinates are millimetres from the sheet's bottom-left corner, y up.

#include "core/Brep.hpp"

#include <map>
#include <string>
#include <vector>

namespace sicdm {

class Scene;
struct HoleNote;

enum class ViewKind { Front, Back, Top, Bottom, Left, Right, Iso, Section, Flat };
const char* viewKindName(ViewKind k);
// A standard view's axes: right and up on the sheet, and into the scene (away from the viewer).
void viewAxes(ViewKind k, DVec3& right, DVec3& up, DVec3& into);
bool viewKindFromName(const std::string& name, ViewKind* out);

struct DrawingView {
    std::string name;
    ViewKind kind = ViewKind::Front;
    double x = 0, y = 0;      // the view's centre on the sheet (mm)
    double scale = 0;         // 0: the drawing's
    bool hidden = true;       // hidden lines dashed (off: left out)
    // Section views: cut through `parent` on a line across it at `cut` (model mm along
    // that view's horizontal axis when `vertical`, its vertical axis otherwise), looking
    // the way the arrows on the cut line point.
    std::string parent;
    bool vertical = true;
    double cut = 0;
    bool flip = false;        // look the other way along the cut
    // Flat views: the flat pattern of this sheet metal body, seen from the side the base's
    // normal points to. Its model points are flat-pattern coordinates (x, y on the base's
    // plane, z through the sheet).
    std::string body;
};

// A size tolerance on a dimension.
struct DimTolerance {
    enum Kind { None, Symmetric, Deviation, Limits, Fit, Basic } kind = None;
    double upper = 0, lower = 0; // mm from the nominal size (Symmetric: +/- upper)
    std::string fit;             // Fit: an ISO 286 code, H7, g6 ...
    bool values = false;         // Fit: its limits too, (+0.021/0)
};

struct DrawingDim {
    enum Kind { Horizontal, Vertical, Aligned, Diameter, Radius } kind = Horizontal;
    std::string view;
    DVec3 a, b;               // model points; Diameter / Radius: a the centre, b on the circle
    double offset = 10;       // sheet mm from the measured feature
    std::string text;         // instead of the measured value
    DimTolerance tol;
};

// The geometric characteristics (ISO 1101 / ASME Y14.5).
enum class GdtSymbol {
    Straightness, Flatness, Circularity, Cylindricity,          // form: no datums
    LineProfile, SurfaceProfile,                                // profile: datums optional
    Angularity, Perpendicularity, Parallelism,                  // orientation
    Position, Concentricity, Symmetry,                          // location
    CircularRunout, TotalRunout                                 // runout
};
const char* gdtName(GdtSymbol s);
bool gdtFromName(const std::string& name, GdtSymbol* out);
bool gdtTakesDatums(GdtSymbol s); // form tolerances take none
bool gdtNeedsDatum(GdtSymbol s);  // orientation, location and runout need one
bool gdtTakesDiameter(GdtSymbol s); // a cylindrical zone makes sense

// A feature control frame: | symbol | [dia] tolerance [M|L] | A | B | C |, on a leader to a
// model point in a view, or under a dimension's value.
struct DrawingFrame {
    GdtSymbol symbol = GdtSymbol::Position;
    double tolerance = 0.1;      // mm
    bool diameter = false;       // a cylindrical zone (the diameter sign before the value)
    char material = 0;           // 'M' maximum or 'L' least material condition, or 0
    std::vector<std::string> datums; // up to three, primary first; "B(M)" carries a modifier
    std::string view;            // on a leader to `at` in this view...
    DVec3 at;
    double dx = 14, dy = 10;     // ... the frame this far from the point (sheet mm)
    int dim = -1;                // ... or under dimension `dim` (index into dims)
};

// A datum feature symbol: the letter in a box, on a leader ending in a filled triangle on
// the feature.
struct DrawingDatum {
    std::string letter;
    std::string view;
    DVec3 at;
    double dx = 0, dy = -12;
};

// A surface texture mark (ISO 1302): the check mark on the surface, its Ra over the long leg.
struct DrawingFinish {
    double ra = 1.6;             // micrometres
    int process = 0;             // 0 any, 1 material removal required, 2 removal not allowed
    std::string view;
    DVec3 at;
    double dx = 0, dy = 0;       // the mark's foot from the point (sheet mm)
};

// A hole callout: a leader from the hole's edge to its description, one line per '\n'
// ("4X \xC3\x98" "6.6 THRU"). {DEPTH}, {CBORE} and {CSINK} in the text draw those symbols.
struct DrawingCallout {
    std::string view;
    DVec3 at;              // the hole's axis where it opens (model)
    double radius = 0;     // model mm: the leader starts on this circle
    std::string text;
    double dx = 0, dy = 0; // where the text starts from the hole's centre, sheet mm (0, 0: up and to the right)
};

struct DrawingNote {
    double x = 0, y = 0;
    double height = 3.5;
    std::string text;
};

struct Drawing {
    std::string sheet = "A3"; // A4..A0, LETTER, TABLOID (ANSI A, B), ANSI C, ANSI D
    bool landscape = true;
    bool thirdAngle = false;  // ISO first angle by default
    double scale = 1.0;       // sheet mm per model mm
    std::vector<std::string> bodies; // empty: every visible body
    std::vector<DrawingView> views;
    std::vector<DrawingDim> dims;
    std::vector<DrawingNote> notes;
    std::vector<DrawingFrame> frames;
    std::vector<DrawingDatum> datums;
    std::vector<DrawingFinish> finishes;
    std::vector<DrawingCallout> callouts;
    std::string generalTolerance; // "ISO 2768-mK": shown in the title block
    std::map<std::string, std::string> title; // TITLE, PART, MATERIAL, DRAWN, CHECKED, COMPANY, REV, DATE
    bool empty() const { return views.empty(); }
    const DrawingView* view(const std::string& name) const;
};

// Sheet sizes in mm (landscape width, height); false for unknown names.
bool sheetSize(const std::string& name, double& w, double& h);
// "1:2", "2:1", "1" -> sheet mm per model mm.
bool parseScale(const std::string& s, double& scale);
std::string scaleText(double scale);
// A standard scale that fits `needed` (sheet mm per model mm) or less.
double standardScale(double needed);

// --- what the sheet looks like ------------------------------------------------------------

struct SheetLine {
    double x0, y0, x1, y1;
    enum Style : unsigned char { Visible, Hidden, Center, Thin, Border, Cut } style = Visible;
};
struct SheetText {
    double x = 0, y = 0, height = 3.5, angle = 0; // degrees, counter-clockwise
    int align = 0;                                // 0 left, 1 centre, 2 right (along the baseline)
    std::string text;
};
struct SheetPoint { // a point the dimension tool snaps to
    double x, y;
    std::string view;
    DVec3 model;
};
struct SheetGraphics {
    double width = 0, height = 0;
    std::vector<SheetLine> lines;
    std::vector<SheetText> texts;
    std::vector<std::vector<std::pair<double, double>>> fills; // arrowheads (filled polygons)
    std::vector<SheetPoint> snaps;
    std::vector<std::string> warnings;
};

// Draws the drawing of the scene's bodies. `quality`: 1 normal, more for finer
// hidden-line tests (exports use 2).
SheetGraphics renderDrawing(const Drawing& d, const Scene& scene, int quality = 1);

// A new drawing of the bodies: front, top, right and isometric views laid out on the
// sheet, at a standard scale that fits.
Drawing defaultDrawing(const Scene& scene, const std::vector<std::string>& bodies, const std::string& sheet, bool landscape, bool thirdAngle,
                       double scale = 0);
// Where a standard view goes next to the front view (first or third angle).
void placeView(Drawing& d, const Scene& scene, DrawingView& v);
// Overall sizes of each orthographic view, and the diameters of round holes and bosses
// seen end on. Returns how many were added.
int autoDimension(Drawing& d, const Scene& scene, const std::string& onlyView = "");
// Callouts for the holes seen end on in the front, top and side views: hole wizard holes
// as they were made (drill, depth, counterbore, countersink, thread), other round holes
// from the geometry (diameter, depth or THRU, counterbore). Holes alike in a view share
// one callout ("4X"); their plain diameter dimensions go. Returns how many were added.
int autoCallouts(Drawing& d, const Scene& scene, const std::string& onlyView = "");
// A hole wizard hole's callout, line by line: "\xC3\x98" "5 {DEPTH}15\nM6x1 - 6H {DEPTH}12".
std::string holeCalloutText(const HoleNote& h);
// The sheet metal bodies a flat view can show (body kind "sheet").
bool isSheetMetalBody(const Scene& scene, const std::string& body);

// The measured value of a dimension (model mm).
double dimensionValue(const Drawing& d, const DrawingDim& dim);
// A dimension's text: its value (or text), with its tolerance. `stack` gets the upper and
// lower lines of a stacked tolerance (deviations, limits) when there is one.
std::string dimensionText(const Drawing& d, const DrawingDim& dim, std::string* upper = nullptr, std::string* lower = nullptr);
// The limits a dimension's tolerance allows (mm): false when it has none (or a basic one).
bool dimensionLimits(const Drawing& d, const DrawingDim& dim, double& high, double& low);

std::string sheetToSvg(const SheetGraphics& g);
std::string sheetToPdf(const SheetGraphics& g, const std::string& title = "Drawing");
std::string sheetToDxf(const SheetGraphics& g);

class Json;
Json drawingToJson(const Drawing& d);
bool drawingFromJson(const Json& j, Drawing& d);

} // namespace sicdm

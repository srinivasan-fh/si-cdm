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

enum class ViewKind { Front, Back, Top, Bottom, Left, Right, Iso, Section };
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
};

struct DrawingDim {
    enum Kind { Horizontal, Vertical, Aligned, Diameter, Radius } kind = Horizontal;
    std::string view;
    DVec3 a, b;               // model points; Diameter / Radius: a the centre, b on the circle
    double offset = 10;       // sheet mm from the measured feature
    std::string text;         // instead of the measured value
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
// The measured value of a dimension (model mm).
double dimensionValue(const Drawing& d, const DrawingDim& dim);

std::string sheetToSvg(const SheetGraphics& g);
std::string sheetToPdf(const SheetGraphics& g, const std::string& title = "Drawing");
std::string sheetToDxf(const SheetGraphics& g);

class Json;
Json drawingToJson(const Drawing& d);
bool drawingFromJson(const Json& j, Drawing& d);

} // namespace sicdm

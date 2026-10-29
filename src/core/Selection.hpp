#pragma once

// More than one body selected: the selected body (Document::selectedBody, the active one the
// gizmo and single-body commands act on) and others with it, picked by Ctrl/Cmd+click, a
// box, or SELECT ADD. The others are kept by name, so deleting or adding bodies doesn't
// mix them up, and only while the active body is still the one they were picked with:
// anything that selects another body on its own (a new body, a click) drops them.

#include <string>
#include <vector>

namespace sicdm {

class Scene;
class Sketch;
struct Body;
struct SketchCurve;

struct BodySelection {
    std::vector<std::string> others; // besides the active body
    std::string anchor;              // the active body's name when they were picked
};

// The selected bodies: the active one first (when there is one), then the others still there.
std::vector<int> selectedBodies(const Scene& scene, int active, const BodySelection& selection);

// Makes exactly these the selection (the first the active one; none: nothing selected).
void selectBodies(const Scene& scene, int& active, BodySelection& selection, const std::vector<int>& bodies);

// Adds a body (it becomes the active one), or takes it out when it was selected
// (Ctrl/Cmd+click). Hidden bodies can be selected (from the model tree).
void toggleBody(const Scene& scene, int& active, BodySelection& selection, int body);

// Bodies copied (COPY, CUT), for PASTE: one for the whole program, so they can be pasted
// into another open file.
std::vector<Body>& bodyClipboard();

// A name not used in the scene (nor in `taken`): `base`, else "base copy", "base copy 2"...
std::string copyName(const Scene& scene, const std::string& base, const std::vector<std::string>& taken = {}, bool keepIfFree = false);

// The same for sketch curves: several selected (the active one is Document::selectedCurve),
// kept by name; a clipboard of them; a name not used by another curve.
std::vector<int> selectedCurves(const Sketch& sketch, int active, const BodySelection& selection);
void selectCurves(const Sketch& sketch, int& active, BodySelection& selection, const std::vector<int>& curves);
void toggleCurve(const Sketch& sketch, int& active, BodySelection& selection, int curve);
std::vector<SketchCurve>& curveClipboard();
std::string copyCurveName(const Sketch& sketch, const std::string& base, const std::vector<std::string>& taken = {}, bool keepIfFree = false);

} // namespace sicdm

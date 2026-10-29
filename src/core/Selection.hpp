#pragma once

// More than one body selected: the selected body (Document::selectedBody, the active one the
// gizmo and single-body commands act on) and others with it, picked by Shift / Ctrl+click, a
// box, or SELECT ADD. The others are kept by name, so deleting or adding bodies doesn't
// mix them up, and only while the active body is still the one they were picked with:
// anything that selects another body on its own (a new body, a click) drops them.

#include <string>
#include <vector>

namespace sicdm {

class Scene;

struct BodySelection {
    std::vector<std::string> others; // besides the active body
    std::string anchor;              // the active body's name when they were picked
};

// The selected bodies: the active one first (when there is one), then the others still there.
std::vector<int> selectedBodies(const Scene& scene, int active, const BodySelection& selection);

// Makes exactly these the selection (the first the active one; none: nothing selected).
void selectBodies(const Scene& scene, int& active, BodySelection& selection, const std::vector<int>& bodies);

// Adds a body (it becomes the active one), or takes it out when it was selected
// (Shift / Ctrl+click). Hidden bodies can be selected (from the model tree).
void toggleBody(const Scene& scene, int& active, BodySelection& selection, int body);

} // namespace sicdm

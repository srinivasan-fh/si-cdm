#pragma once

#include "core/Camera.hpp"
#include "core/FileIO.hpp"
#include "core/Scene.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sicdm {

// The open document: model, sketch plane, file and selection. Shared by the
// GUI, the command line and the MCP server so they all edit the same thing.
struct Document {
    Scene scene;
    ProjectState state;
    std::string path; // project file, empty until saved
    bool dirty = false;

    int selectedBody = -1;
    int selectedCurve = -1;
    int selectedImage = -1;

    bool snapGrid = true;
    bool snapPoints = true;

    int primitiveSerial = 0;

    void clearSelection() noexcept { selectedBody = selectedCurve = selectedImage = -1; }
    std::string title() const; // "name.sicdm" or "Untitled", with " *" when dirty
};

// Window / camera operations. Implemented by the GUI; absent when headless.
class ViewHost {
public:
    virtual ~ViewHost() = default;
    virtual void openWindow(bool drawing2D) = 0;
    virtual void standardView(StandardView v) = 0; // 2D window: picks the drawing plane
    virtual void planeChanged(SketchPlane p) = 0;  // re-orient the 2D window
    virtual void fit() = 0;
    virtual void zoomStep(float steps) = 0;
    virtual void home() = 0;
    virtual void roll(float degrees, bool absolute) = 0;
    virtual void setProjection(Projection p) = 0;
    virtual void setDisplay(int mode) = 0; // 0 shaded, 1 shaded+edges, 2 wireframe, 3 hidden line
    virtual void setGrid(std::optional<bool> on) = 0;  // nullopt toggles
    virtual void setPanel(std::optional<bool> on) = 0; // Model panel
    virtual std::string describe() const = 0;          // window, plane, camera summary
};

struct CommandOutput {
    bool ok = true;
    std::vector<std::string> lines;
    bool clearHistory = false;
    bool documentReplaced = false; // NEW / OPEN: views should refit

    void say(std::string s) { lines.push_back(std::move(s)); }
    CommandOutput& fail(std::string s) {
        ok = false;
        lines.push_back(std::move(s));
        return *this;
    }
};

// Runs one command line (see commandTable()) against the document.
CommandOutput executeCommand(Document& doc, std::string_view line, ViewHost* view);

// Default-sized primitive for the tool rail ("box", "cylinder", ...), named "<Kind> <serial>".
Body defaultPrimitive(const std::string& kind, int serial);

// Text summary of the document (bodies, curves, images) used by LIST and MCP.
std::vector<std::string> describeDocument(const Document& doc);

} // namespace sicdm

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
    std::string name = "Untitled"; // shown while there is no path
    bool dirty = false;

    int selectedBody = -1;
    int selectedCurve = -1;
    int selectedImage = -1;

    // Commands built by the MCP tools pass plain numbers in millimetres whatever the
    // document's display units are; set while such a tool runs.
    bool apiMillimeters = false;
    // Set while REGEN replays the history (the replayed commands aren't recorded again).
    bool replaying = false;
    int replayStep = -1;            // the history step being replayed
    std::string pendingConstraint;  // a constraint just made: it learns its history step
    bool historyRewritten = false;  // the command changed the history itself: it isn't recorded

    bool snapGrid = true;
    bool snapPoints = true;

    int primitiveSerial = 0;

    void clearSelection() noexcept { selectedBody = selectedCurve = selectedImage = -1; }
    std::string title() const; // "file.sicdm" or the name, with " *" when dirty
    std::string displayName() const; // "file.sicdm" or the name
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

    // Multi-document hosts keep each project in its own file tab. NEW / OPEN call
    // prepareDocument() before they fill the document: the host may make a fresh tab
    // active (the current file stays open in its tab). It returns true when `path`
    // is already open and its tab was activated instead, so there is nothing to load.
    virtual bool prepareDocument(const std::string& path) { (void)path; return false; }
    // Simulation window: "open", "play", "pause", "toggle", "stop", "time", "speed",
    // "axes", "paths" (value: seconds, factor or 0/1). False when headless.
    virtual bool simulation(const std::string& action, float value, std::string* reply) {
        (void)action;
        (void)value;
        (void)reply;
        return false;
    }
    virtual std::vector<std::string> fileTabs() const { return {}; } // "> " marks the active one
    virtual bool switchFile(const std::string& which, std::string* error) {
        *error = "Only one document is open";
        (void)which;
        return false;
    }
    virtual bool closeFile(bool discard, std::string* error) {
        *error = "CLOSE needs the app window";
        (void)discard;
        return false;
    }
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

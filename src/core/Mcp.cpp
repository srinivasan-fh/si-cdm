#include "core/Mcp.hpp"
#include "core/Units.hpp"

#include "core/Command.hpp"
#include "core/Library.hpp"
#include "core/Physics.hpp"
#include "core/Pose.hpp"

#include <algorithm>
#include <cstdio>

namespace sicdm {

namespace {

constexpr const char* kServerVersion = "0.1.0";
constexpr const char* kSupportedVersions[] = {"2025-06-18", "2025-03-26", "2024-11-05"};

constexpr const char* kInstructions =
    "SI CDM is a CAD/CAM workbench. Tool arguments and get_scene numbers are millimetres; Z is up. "
    "run_command reads bare numbers in the document's display units (get_scene project.display_units; "
    "change them with UNITS) and accepts explicit units anywhere: 12.5mm 2cm 0.5m 3in 1/8\" 5'6-1/2\" 10pt. The model has 3D bodies "
    "(parametric solids or imported meshes), 2D sketch curves on the Top (XY), Front (XZ) or Right (YZ) "
    "plane, and reference images on those planes. Call get_scene first to see what exists. "
    "Sketch coordinates are 2D in the plane: Top u=X v=Y, Front u=X v=Z, Right u=Y v=Z. "
    "To make solids from sketches, draw a closed curve (rectangle, circle, closed polyline) with create_sketch, "
    "then call extrude with its name and a distance. "
    "Use run_command for anything not covered by a dedicated tool (call list_commands for the syntax).";

Json errorReply(const Json& id, int code, const std::string& message) {
    Json e = Json::object();
    e.set("code", code).set("message", message);
    Json r = Json::object();
    r.set("jsonrpc", "2.0").set("id", id).set("error", e);
    return r;
}

Json resultReply(const Json& id, Json result) {
    Json r = Json::object();
    r.set("jsonrpc", "2.0").set("id", id).set("result", std::move(result));
    return r;
}

Json textResult(const std::string& text, bool isError) {
    Json item = Json::object();
    item.set("type", "text").set("text", text);
    Json r = Json::object();
    r.set("content", Json(Json::Array{item})).set("isError", isError);
    return r;
}

// JSON-schema helpers.
Json prop(const char* type, const char* description) {
    Json p = Json::object();
    p.set("type", type).set("description", description);
    return p;
}
Json enumProp(std::initializer_list<const char*> values, const char* description) {
    Json p = prop("string", description);
    Json e = Json::array();
    for (const char* v : values) e.push(v);
    p.set("enum", e);
    return p;
}
Json numberArray(int n, const char* description) {
    Json p = prop("array", description);
    Json items = Json::object();
    items.set("type", "number");
    p.set("items", items).set("minItems", n).set("maxItems", n);
    return p;
}
Json tool(const char* name, const char* description, Json properties, std::initializer_list<const char*> required,
          bool readOnly = false) {
    Json schema = Json::object();
    schema.set("type", "object").set("properties", std::move(properties));
    Json req = Json::array();
    for (const char* r : required) req.push(r);
    schema.set("required", req);
    Json t = Json::object();
    t.set("name", name).set("description", description).set("inputSchema", schema);
    Json ann = Json::object();
    ann.set("readOnlyHint", readOnly);
    t.set("annotations", ann);
    return t;
}

std::string quote(const std::string& s) { return "\"" + s + "\""; }

std::string num(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
}

std::string pointArg(const Json& p) { return num(p[0].asNumber()) + "," + num(p[1].asNumber()); }

Json vec3(const Vec3& v) { return Json(Json::Array{v.x, v.y, v.z}); }

const char* curveKindKey(CurveKind k) {
    switch (k) {
    case CurveKind::Point: return "point";
    case CurveKind::Line: return "line";
    case CurveKind::Polyline: return "polyline";
    case CurveKind::Rectangle: return "rectangle";
    case CurveKind::Circle: return "circle";
    }
    return "line";
}

} // namespace

Json McpServer::toolDefinitions() {
    Json tools = Json::array();
    tools.push(tool("get_scene",
                    "Return the whole document as JSON: bodies (kind, parameters, position, bounds), sketch curves "
                    "(plane, points, radius), reference images, active sketch plane, selection, file and view state.",
                    Json::object(), {}, true));
    tools.push(tool("list_commands", "List every command-line command with its syntax.", Json::object(), {}, true));
    {
        Json p = Json::object();
        p.set("command", prop("string", "One SI CDM command line, e.g. 'CIRCLE 0,0 25' or 'VIEW FRONT'"));
        tools.push(tool("run_command", "Run any SI CDM command line and return its output.", p, {"command"}));
    }
    {
        Json p = Json::object();
        p.set("kind", enumProp({"point", "line", "polyline", "rectangle", "circle"}, "Curve type"));
        Json pts = prop("array", "2D points [[u,v], ...] in plane mm. line: 2 or more (connected segments); "
                                 "rectangle: 2 opposite corners; circle: centre; point: 1");
        Json pair = Json::object();
        pair.set("type", "array");
        Json num1 = Json::object();
        num1.set("type", "number");
        pair.set("items", num1).set("minItems", 2).set("maxItems", 2);
        pts.set("items", pair);
        p.set("points", pts);
        p.set("radius", prop("number", "Circle radius (mm)"));
        p.set("plane", enumProp({"top", "front", "right"}, "Sketch plane (default: active plane)"));
        p.set("closed", prop("boolean", "Polyline: close back to the first point"));
        tools.push(tool("create_sketch", "Draw a 2D sketch curve on a sketch plane.", p, {"kind", "points"}));
    }
    {
        Json p = Json::object();
        p.set("kind", enumProp({"box", "cylinder", "cone", "sphere", "torus", "wedge", "pyramid", "prism"}, "Solid type"));
        p.set("width", prop("number", "box / wedge: size along X"));
        p.set("depth", prop("number", "box / wedge: size along Y"));
        p.set("height", prop("number", "box, wedge, cylinder, cone, pyramid, prism: size along Z"));
        p.set("radius", prop("number", "cylinder, cone (base), sphere, pyramid, prism (circumradius); torus: ring radius"));
        p.set("top_radius", prop("number", "cone: top radius (0 = pointed)"));
        p.set("tube_radius", prop("number", "torus: tube radius"));
        p.set("sides", prop("integer", "pyramid / prism: number of sides (3-64)"));
        p.set("position", numberArray(3, "[x,y,z] of the base centre (mm)"));
        p.set("name", prop("string", "Optional name"));
        p.set("color", numberArray(3, "Optional [r,g,b] in 0..1"));
        tools.push(tool("create_solid", "Create a parametric 3D solid. Bases sit at position.z and are centred on x,y.",
                        p, {"kind"}));
    }
    {
        Json p = Json::object();
        p.set("profile", prop("string", "Name of a closed sketch curve: rectangle, circle or closed polyline"));
        p.set("distance", prop("number", "Depth in mm along the sketch-plane normal (negative = other side)"));
        p.set("symmetric", prop("boolean", "Extrude half the distance to each side of the plane"));
        p.set("keep_sketch", prop("boolean", "Keep the profile curve visible (default: hidden after extruding)"));
        p.set("name", prop("string", "Optional name for the new body"));
        tools.push(tool("extrude", "Extrude a closed 2D sketch curve into a 3D solid.", p, {"profile", "distance"}));
    }
    {
        Json p = Json::object();
        p.set("name", prop("string", "Body, curve or image name (see get_scene)"));
        p.set("position", numberArray(3, "Body: new [x,y,z] position; image: [u,v,ignored] centre"));
        p.set("move_by", numberArray(3, "Body: [dx,dy,dz]; image: [du,dv,ignored]"));
        p.set("rename", prop("string", "New name"));
        p.set("visible", prop("boolean", "Show or hide"));
        p.set("color", numberArray(3, "Body colour [r,g,b] in 0..1"));
        p.set("width", prop("number", "Image width (mm)"));
        p.set("opacity", prop("number", "Image opacity 0..1"));
        p.set("scale", prop("number", "Body: size factor (a library component scales as a whole, with its motions)"));
        Json params = Json::object();
        params.set("type", "object").set("description",
            "Library part parameters, e.g. {\"shafts\": \"dual\", \"shaft_length\": 30, \"rear_length\": 12} for a "
            "motor (name the motor or its rotor) or {\"teeth\": 32} for a gear; see the PARAM command");
        p.set("params", params);
        tools.push(tool("modify", "Move, rename, recolour, show/hide a body, curve or image.", p, {"name"}));
    }
    {
        Json p = Json::object();
        p.set("name", prop("string", "Name of the body, curve or image to delete"));
        tools.push(tool("delete", "Delete a body, sketch curve or image.", p, {"name"}));
    }
    {
        Json p = Json::object();
        p.set("window", enumProp({"2d", "3d", "simulation"}, "2D Drawing, 3D Model or Simulation window"));
        p.set("plane", enumProp({"top", "front", "right"}, "Sketch / drawing plane"));
        p.set("view", enumProp({"top", "bottom", "front", "back", "left", "right", "iso"}, "Standard view (3D)"));
        p.set("projection", enumProp({"perspective", "orthographic"}, "3D projection"));
        p.set("display", enumProp({"shaded", "edges", "wireframe", "hidden"}, "3D display mode"));
        p.set("roll", prop("number", "Roll the 3D view by degrees (0 levels it)"));
        p.set("fit", prop("boolean", "Zoom to fit everything"));
        tools.push(tool("set_view", "Change the window, plane, camera view or display mode of the app.", p, {}));
    }
    {
        Json keys = Json::array();
        for (const LibraryItem& it : libraryItems()) keys.push(it.key);
        Json p = Json::object();
        Json part = prop("string", "Library part (see list_parts)");
        part.set("enum", keys);
        p.set("part", part);
        p.set("name", prop("string", "Component name (default e.g. \"DC Motor 1\")"));
        p.set("position", numberArray(3, "[x,y,z] (default: beside the model)"));
        Json params = Json::object();
        params.set("type", "object").set("description",
            "Part parameters by key, e.g. {\"size\": 1.5, \"rpm\": 60} or {\"module\": 2, \"teeth\": 24} (see list_parts)");
        p.set("params", params);
        tools.push(tool("insert_part",
                        "Insert a library component: DC / stepper / servo motors (with a 360 deg drive motion), spur, "
                        "helical, herringbone, bevel, internal ring and worm gears, racks, and linked gear trains.",
                        p, {"part"}));
        tools.push(tool("list_parts", "List the library parts with their parameters and defaults.", Json::object(), {}));
    }
    {
        Json p = Json::object();
        p.set("kind", enumProp({"circular", "linear"}, "circular: rotation about an axis; linear: travel along a direction"));
        p.set("name", prop("string", "Motion name; an existing motion with this name is updated"));
        Json bodies = Json::object();
        bodies.set("type", "array").set("items", prop("string", "Body name")).set("description",
            "Bodies moved by it; [\"*\"] = all. Bodies in several motions chain them in list order (first = outermost), "
            "e.g. a linear 'Drive' on all bodies listed before a circular 'Wheel spin' on the wheels");
        p.set("bodies", bodies);
        p.set("axis", numberArray(3, "Rotation axis / travel direction [x,y,z] (default Z for circular, X for linear)"));
        p.set("origin", numberArray(3, "Point on the rotation axis [x,y,z] (default: centre of the bodies)"));
        p.set("each_centre", prop("boolean", "Circular: every body turns about its own centre (e.g. all wheels in one motion)"));
        p.set("speed", prop("number", "deg/s (circular) or mm/s (linear); negative reverses"));
        p.set("rpm", prop("number", "Circular speed in revolutions per minute (instead of speed)"));
        p.set("range", prop("number", "Sweep in degrees / stroke in mm; 0 = unlimited"));
        p.set("mode", enumProp({"continuous", "loop", "oscillate", "once"}, "Behaviour at the end of the range"));
        p.set("start", prop("number", "Delay before it starts (s)"));
        p.set("enabled", prop("boolean", "Enable or disable the motion"));
        p.set("follow", prop("string", "Circular motion that drives this one (gear / pulley / sprocket trains)"));
        p.set("ratio", prop("number", "Circular follower: angle = ratio x the leader's angle"));
        p.set("radius", prop("number", "Linear follower: travel = leader angle (rad) x radius"));
        p.set("along_loop", prop("boolean", "Linear: move a chain / belt body along its own loop instead of translating it"));
        p.set("follow", prop("string", "Linear: circular motion (wheel spin) that drives the travel, rolling without "
                                       "slip; speed / range / mode then come from it. \"\" unlinks"));
        p.set("radius", prop("number", "Linear with follow: wheel radius in mm (0 = from the wheel's size)"));
        tools.push(tool("create_motion",
                        "Add or update a circular or linear motion of bodies, played by the Simulation window.", p,
                        {"kind"}));
        Json d = Json::object();
        d.set("name", prop("string", "Motion to delete"));
        tools.push(tool("delete_motion", "Delete a motion.", d, {"name"}));
        Json s = Json::object();
        s.set("action", enumProp({"open", "play", "pause", "stop", "seek", "speed", "duration", "poses", "axes", "paths"},
                                 "open the Simulation window, control playback, or get body poses at a time"));
        s.set("time", prop("number", "seconds, for seek and poses"));
        s.set("value", prop("number", "speed factor (speed), seconds (duration), 1/0 (axes, paths)"));
        tools.push(tool("simulate",
                        "Run the motion simulation. 'poses' works headless too and reports every motion value and "
                        "moving body position at 'time'.",
                        s, {"action"}));
    }
    {
        Json postures = Json::array(), gestures = Json::array();
        for (const NamedPosture& n : namedPostures()) postures.push(n.name);
        for (const Gesture& g : namedGestures()) gestures.push(g.name);
        Json p = Json::object();
        p.set("robot", prop("string", "Robot component or one of its bodies (default: the selected or first robot)"));
        Json preset = prop("string", "Preset posture");
        preset.set("enum", postures);
        p.set("posture", preset);
        Json joints = Json::object();
        joints.set("type", "object").set("description",
            "Joint angles in degrees by standard id, applied after the preset: {\"right elbow\": 90, \"neck yaw\": -30}. "
            "A group name sets several ({\"elbow\": 45}, {\"right index\": 60}, {\"*\": 0}). Signs: + flexion, "
            "abduction, internal rotation, plantarflexion; neck yaw + turns left");
        p.set("joints", joints);
        p.set("seconds", prop("number", "Move time (0 / omitted: as fast as the motors allow); never faster than their rated speed"));
        tools.push(tool("set_pose",
                        "Move a library robot (humanoid, arm, hand, finger, thumb, leg, head & neck) into a posture. "
                        "Angles are clamped to each joint's standard hard stops, reached on minimum-jerk paths by the "
                        "joints' DC gear motors, and a humanoid keeps its feet on the floor. Returns notes on clamped "
                        "or coupled joints. Use run_command 'POSE robot JOINTS' to read the joint ids and ranges.",
                        p, {}));
        Json g = Json::object();
        g.set("robot", prop("string", "Robot component or one of its bodies (default: the selected or first robot)"));
        Json gesture = prop("string", "Gesture");
        gesture.set("enum", gestures);
        g.set("gesture", gesture);
        g.set("loop", prop("boolean", "Repeat it (default: the gesture's own setting; march loops)"));
        g.set("speed", prop("number", "Playback speed factor (2 = twice as fast, never beyond the motors' rated speed)"));
        g.set("mirror", prop("boolean", "Play it with the other side (left arm / hand)"));
        tools.push(tool("play_gesture", "Play a timed gesture (wave, nod, point, march...) on a library robot from the posture it holds.",
                        g, {"gesture"}));
    }
    {
        Json demos = Json::array();
        for (const PhysicsDemo& d : physicsDemos()) demos.push(d.key);
        Json p = Json::object();
        p.set("enabled", prop("boolean", "Physics on / off for the Simulation"));
        Json demo = prop("string", "Load a ready-made experiment (replaces the scene)");
        demo.set("enum", demos);
        p.set("demo", demo);
        p.set("gravity", enumProp({"off", "uniform", "newton", "both"},
                                  "uniform: g everywhere; newton: every pair of masses attracts (G m1 m2 / r^2)"));
        p.set("g", numberArray(3, "Uniform gravity [x,y,z] m/s^2 (default [0,0,-9.80665])"));
        p.set("big_g", prop("number", "Newton's gravitational constant (default 6.6743e-11)"));
        p.set("floor", prop("boolean", "Solid ground at z = 0"));
        p.set("friction", prop("number", "Floor friction coefficient"));
        p.set("air", prop("boolean", "Air drag"));
        p.set("electric_field", numberArray(3, "Uniform E field [x,y,z] V/m"));
        p.set("magnetic_field", numberArray(3, "Uniform B field [x,y,z] T"));
        p.set("ambient_c", prop("number", "Air temperature, C"));
        p.set("relativity", prop("boolean", "Special relativity"));
        p.set("light_speed", prop("number", "Speed of light, m/s (lower it to see relativistic effects)"));
        p.set("duration", prop("number", "Seconds simulated"));
        Json body = Json::object();
        body.set("type", "object").set("description",
            "One body's physics: {\"name\": \"Ball 1\", \"mode\": \"dynamic\"|\"fixed\"|\"off\", \"mass\": kg, "
            "\"density\": kg/m^3, \"velocity\": [x,y,z] mm/s, \"spin\": [x,y,z] deg/s, \"charge\": C, "
            "\"temperature_c\": C, \"specific_heat\": J/(kg K), \"restitution\": 0-1, \"force\": [x,y,z] N, "
            "\"wave\": \"sound\"|\"light\"|\"custom\"|\"off\", \"wave_frequency\": Hz, \"wave_speed\": mm/s}");
        p.set("body", body);
        tools.push(tool("set_physics",
                        "Set up the physical environment of the Simulation (gravity, floor, air, E / B fields, "
                        "ambient temperature, relativity) and bodies' mass, velocity, charge, temperature, "
                        "applied force and wave sources; or load a demo experiment.",
                        p, {}));
        Json r = Json::object();
        r.set("time", prop("number", "Seconds into the run (default 0)"));
        r.set("body", prop("string", "Only this body (default: all)"));
        tools.push(tool("physics_report",
                        "Physics state at a time: kinematics (position, velocity, acceleration), forces, momentum, "
                        "energy with the conservation check, temperature, relativity (gamma, proper time), quantum "
                        "(de Broglie wavelength) and wave / Doppler readouts. Works headless.",
                        r, {}, true));
    }
    {
        Json p = Json::object();
        p.set("path", prop("string", "File path (.sicdm). Optional for save when the project already has a path"));
        tools.push(tool("save_project", "Save the project.", p, {}));
        Json o = Json::object();
        o.set("path", prop("string", "Project file to open (.sicdm)"));
        tools.push(tool("open_project", "Open a project. In the app it opens in its own file tab (or shows its tab if already open).", o, {"path"}));
        tools.push(tool("new_project", "Start an empty project. In the app it opens a new file tab; headless, unsaved changes are discarded.", Json::object(), {}));
    }
    {
        Json p = Json::object();
        p.set("path", prop("string", ".stl/.obj mesh, .dxf drawing, or image (.png .jpg .bmp .tga .gif ...)"));
        p.set("position", numberArray(2, "Image centre [u,v] on the sketch plane (mm)"));
        p.set("width", prop("number", "Image width (mm)"));
        tools.push(tool("import_file", "Import a mesh as a body, a DXF onto the sketch plane, or a reference image.",
                        p, {"path"}));
        Json e = Json::object();
        e.set("path", prop("string", ".stl/.obj (visible bodies) or .dxf/.svg (curves on the sketch plane)"));
        tools.push(tool("export_file", "Export the model or the active sketch plane.", e, {"path"}));
    }
    return tools;
}

Json McpServer::sceneJson() const {
    Json s = Json::object();
    Json project = Json::object();
    project.set("path", m_doc.path).set("unsaved_changes", m_doc.dirty).set("units", "mm");
    Json du = Json::object();
    du.set("unit", unitInfo(m_doc.state.units.unit).symbol)
        .set("decimals", m_doc.state.units.decimals())
        .set("fractional", m_doc.state.units.fractional())
        .set("description", describeUnits(m_doc.state.units));
    project.set("display_units", du);
    s.set("project", project);
    s.set("sketch_plane", planeKey(m_doc.state.plane));

    Json bodies = Json::array();
    for (std::size_t i = 0; i < m_doc.scene.bodies().size(); ++i) {
        const Body& b = m_doc.scene.bodies()[i];
        const AABB bb = b.bounds();
        Json jb = Json::object();
        Json params = Json::array();
        for (float p : b.params) params.push(p);
        Json bounds = Json::object();
        bounds.set("min", vec3(bb.min)).set("max", vec3(bb.max));
        jb.set("name", b.name).set("kind", b.kind).set("params", params).set("position", vec3(b.offset));
        jb.set("bounds", bounds).set("color", vec3(b.color)).set("visible", b.visible);
        jb.set("triangles", b.mesh.indices.size() / 3);
        jb.set("scale", b.scale);
        if (b.rotation.x != 0.0f || b.rotation.y != 0.0f || b.rotation.z != 0.0f) jb.set("rotation", vec3(b.rotation));
        if (!b.component.empty()) jb.set("component", b.component);
        jb.set("selected", static_cast<int>(i) == m_doc.selectedBody);
        bodies.push(jb);
    }
    s.set("bodies", bodies);

    Json curves = Json::array();
    for (std::size_t i = 0; i < m_doc.scene.sketch().curves().size(); ++i) {
        const SketchCurve& c = m_doc.scene.sketch().curves()[i];
        Json jc = Json::object();
        Json pts = Json::array();
        for (const Vec2& p : c.pts) pts.push(Json(Json::Array{p.x, p.y}));
        jc.set("name", c.name).set("kind", curveKindKey(c.kind)).set("plane", planeKey(c.plane)).set("points", pts);
        if (c.kind == CurveKind::Circle) jc.set("radius", c.radius);
        jc.set("measure", c.measure()).set("visible", c.visible);
        jc.set("selected", static_cast<int>(i) == m_doc.selectedCurve);
        curves.push(jc);
    }
    s.set("sketch_curves", curves);

    Json images = Json::array();
    for (const ImageRef& img : m_doc.scene.images()) {
        Json ji = Json::object();
        ji.set("name", img.name).set("path", img.path).set("plane", planeKey(img.plane));
        ji.set("center", Json(Json::Array{img.center.x, img.center.y}));
        ji.set("width", img.width).set("height", img.height()).set("opacity", img.opacity);
        ji.set("pixels", Json(Json::Array{img.pixelWidth, img.pixelHeight})).set("visible", img.visible);
        images.push(ji);
    }
    s.set("images", images);

    Json motions = Json::array();
    for (const Motion& m : m_doc.scene.motions()) {
        Json jm = Json::object();
        Json targets = Json::array();
        for (const std::string& b : m.bodies) targets.push(b);
        jm.set("name", m.name).set("kind", motionKindKey(m.kind)).set("bodies", targets);
        jm.set("axis", vec3(m.axis)).set("origin", vec3(m.origin)).set("each_centre", m.eachCentre).set("speed", m.speed);
        jm.set("speed_unit", m.rateUnit()).set("range", m.range).set("mode", motionModeKey(m.mode));
        jm.set("start", m.start).set("enabled", m.enabled);
        if (!m.follow.empty()) jm.set("follow", m.follow).set("ratio", m.ratio).set("radius", m.radius);
        if (m.step > 0.0f) jm.set("step", m.step);
        if (m.alongLoop) jm.set("along_loop", true);
        motions.push(jm);
    }
    s.set("motions", motions);
    s.set("view", m_view ? m_view->describe() : std::string("headless (no window)"));
    return s;
}

Json McpServer::runLine(const std::string& line) {
    const CommandOutput o = executeCommand(m_doc, line, m_view);
    if (o.ok) m_changed = true;
    if (o.documentReplaced) m_replaced = true;
    std::string text;
    for (const std::string& l : o.lines) text += (text.empty() ? "" : "\n") + l;
    if (text.empty()) text = o.ok ? "OK" : "Failed";
    return textResult(text, !o.ok);
}

Json McpServer::callTool(const std::string& name, const Json& a) {
    if (name == "get_scene") {
        Json r = textResult(sceneJson().dump(), false);
        r.set("structuredContent", sceneJson());
        return r;
    }
    if (name == "list_commands") return runLine("HELP");
    if (name == "run_command") {
        if (!a["command"].isString()) return textResult("'command' must be a string", true);
        return runLine(a["command"].asString());
    }
    if (name == "create_sketch") {
        const std::string kind = a["kind"].asString();
        const auto& pts = a["points"].items();
        if (pts.empty()) return textResult("'points' is required", true);
        for (const Json& p : pts)
            if (!p.isArray() || p.size() < 2 || !p[0].isNumber() || !p[1].isNumber())
                return textResult("each point must be [u, v]", true);
        const SketchPlane old = m_doc.state.plane;
        if (a["plane"].isString()) {
            const auto p = planeFromKey(a["plane"].asString());
            if (!p) return textResult("plane must be top, front or right", true);
            m_doc.state.plane = *p;
        }
        std::string line;
        if (kind == "point") line = "POINT " + pointArg(pts[0]);
        else if (kind == "circle") {
            if (!a["radius"].isNumber()) return textResult("circle needs 'radius'", true);
            line = "CIRCLE " + pointArg(pts[0]) + " " + num(a["radius"].asNumber());
        } else if (kind == "line" || kind == "polyline" || kind == "rectangle") {
            line = kind == "line" ? "LINE" : (kind == "polyline" ? "PLINE" : "RECT");
            for (const Json& p : pts) line += " " + pointArg(p);
            if (kind == "polyline" && a["closed"].asBool()) line += " CLOSE";
        } else {
            m_doc.state.plane = old;
            return textResult("kind must be point, line, polyline, rectangle or circle", true);
        }
        Json r = runLine(line);
        m_doc.state.plane = old; // a one-off plane does not change the user's active plane
        return r;
    }
    if (name == "create_solid") {
        const std::string kind = a["kind"].asString();
        auto n = [&](const char* key) { return a[key].isNumber() ? num(a[key].asNumber()) : std::string(); };
        std::string dims;
        auto needAll = [&](std::initializer_list<const char*> keys) {
            for (const char* k : keys) {
                if (!a[k].isNumber()) return std::string("missing '") + k + "'";
                dims += " " + n(k);
            }
            return std::string();
        };
        std::string missing;
        if (kind == "box" || kind == "wedge") missing = needAll({"width", "depth", "height"});
        else if (kind == "cylinder") missing = needAll({"radius", "height"});
        else if (kind == "cone") {
            missing = needAll({"radius", "height"});
            if (a["top_radius"].isNumber()) dims += " " + n("top_radius");
        } else if (kind == "sphere") missing = needAll({"radius"});
        else if (kind == "torus") missing = needAll({"radius", "tube_radius"});
        else if (kind == "pyramid" || kind == "prism") {
            missing = needAll({"radius", "height"});
            if (a["sides"].isNumber()) dims += " " + n("sides");
        } else
            return textResult("unknown solid kind", true);
        if (!missing.empty()) return textResult(kind + ": " + missing, true);
        std::string line = (kind == "box" ? std::string("BOX") : toUpper(kind)) + dims;
        if (a["position"].isArray())
            line += " @" + num(a["position"][0].asNumber()) + "," + num(a["position"][1].asNumber()) + "," +
                    num(a["position"][2].asNumber());
        Json r = runLine(line);
        if (!r["isError"].asBool() && !m_doc.scene.bodies().empty()) {
            Body& b = m_doc.scene.bodies().back();
            if (a["name"].isString() && !a["name"].asString().empty()) b.name = a["name"].asString();
            if (a["color"].isArray()) b.color = {a["color"][0].asFloat(), a["color"][1].asFloat(), a["color"][2].asFloat()};
            r = textResult("Created " + b.name, false);
        }
        return r;
    }
    if (name == "modify") {
        const std::string target = a["name"].asString();
        auto& bodies = m_doc.scene.bodies();
        auto& curves = m_doc.scene.sketch().curves();
        auto& images = m_doc.scene.images();
        auto rename = [&](std::string& n) {
            if (a["rename"].isString() && !a["rename"].asString().empty()) n = a["rename"].asString();
        };
        auto v3 = [&](const char* key) {
            const Json& j = a[key];
            return Vec3{j[0].asFloat(), j[1].asFloat(), j[2].asFloat()};
        };
        if (a["params"].isObject() && !a["params"].members().empty()) {
            std::string line = "PARAM " + quote(target);
            for (const auto& [k, v] : a["params"].members()) {
                if (!v.isNumber() && !v.isString()) return textResult("parameter '" + k + "' must be a number or a choice name", true);
                line += " " + k + " " + (v.isNumber() ? num(v.asNumber()) : quote(v.asString()));
            }
            const Json r = runLine(line);
            if (r["isError"].asBool()) return r;
            bool other = a["scale"].isNumber() || a["position"].isArray() || a["move_by"].isArray() || a["rename"].isString() ||
                         a["visible"].isBool() || a["color"].isArray();
            if (!other) return r;
        }
        if (a["scale"].isNumber()) {
            if (a["scale"].asNumber() <= 0) return textResult("scale must be > 0", true);
            for (Body& b : bodies)
                if (b.name == target) {
                    const Json r = runLine("SCALE " + quote(target) + " TO " + num(a["scale"].asNumber()));
                    if (r["isError"].asBool()) return r;
                    break;
                }
        }
        for (Body& b : bodies)
            if (b.name == target) {
                if (a["position"].isArray()) b.offset = v3("position");
                if (a["move_by"].isArray()) b.offset += v3("move_by");
                if (a["visible"].isBool()) b.visible = a["visible"].asBool();
                if (a["color"].isArray()) b.color = v3("color");
                rename(b.name);
                m_doc.dirty = m_changed = true;
                return textResult("Updated body " + b.name, false);
            }
        for (SketchCurve& c : curves)
            if (c.name == target) {
                if (a["move_by"].isArray())
                    for (Vec2& p : c.pts) p = {p.x + a["move_by"][0].asFloat(), p.y + a["move_by"][1].asFloat()};
                if (a["visible"].isBool()) c.visible = a["visible"].asBool();
                rename(c.name);
                m_doc.scene.sketch().touch();
                m_doc.dirty = m_changed = true;
                return textResult("Updated curve " + c.name, false);
            }
        for (ImageRef& img : images)
            if (img.name == target) {
                if (a["position"].isArray()) img.center = {a["position"][0].asFloat(), a["position"][1].asFloat()};
                if (a["move_by"].isArray())
                    img.center = {img.center.x + a["move_by"][0].asFloat(), img.center.y + a["move_by"][1].asFloat()};
                if (a["width"].isNumber() && a["width"].asNumber() > 0) img.width = a["width"].asFloat();
                if (a["opacity"].isNumber()) img.opacity = std::clamp(a["opacity"].asFloat(), 0.0f, 1.0f);
                if (a["visible"].isBool()) img.visible = a["visible"].asBool();
                rename(img.name);
                m_doc.dirty = m_changed = true;
                return textResult("Updated image " + img.name, false);
            }
        return textResult("Nothing named '" + target + "' (see get_scene)", true);
    }
    if (name == "delete") return runLine("ERASE " + a["name"].asString());
    if (name == "extrude") {
        if (!a["profile"].isString() || !a["distance"].isNumber())
            return textResult("extrude needs 'profile' (curve name) and 'distance'", true);
        std::string line = "EXTRUDE " + a["profile"].asString() + " " + num(a["distance"].asNumber());
        if (a["symmetric"].asBool()) line += " SYMMETRIC";
        if (a["keep_sketch"].asBool()) line += " KEEP";
        Json r = runLine(line);
        if (!r["isError"].asBool() && a["name"].isString() && !a["name"].asString().empty() &&
            !m_doc.scene.bodies().empty()) {
            m_doc.scene.bodies().back().name = a["name"].asString();
            r = textResult(r["content"][0]["text"].asString() + " (named " + a["name"].asString() + ")", false);
        }
        return r;
    }
    if (name == "set_view") {
        std::string log;
        auto step = [&](const std::string& line) {
            const Json r = runLine(line);
            log += (log.empty() ? "" : "\n") + r["content"][0]["text"].asString();
            return !r["isError"].asBool();
        };
        bool ok = true;
        if (a["plane"].isString()) ok = step("PLANE " + a["plane"].asString()) && ok;
        if (a["window"].isString()) {
            const std::string w = a["window"].asString();
            ok = step(w == "2d" ? "2D" : w == "simulation" ? "SIM" : "3D") && ok;
        }
        if (a["view"].isString()) ok = step("VIEW " + a["view"].asString()) && ok;
        if (a["projection"].isString()) ok = step(a["projection"].asString() == "orthographic" ? "ORTHO" : "PERSP") && ok;
        if (a["display"].isString()) ok = step("DISPLAY " + a["display"].asString()) && ok;
        if (a["roll"].isNumber()) ok = step("ROLL " + num(a["roll"].asNumber())) && ok;
        if (a["fit"].asBool()) ok = step("FIT") && ok;
        if (log.empty()) return textResult("Nothing to change", true);
        return textResult(log, !ok);
    }
    if (name == "set_pose" || name == "play_gesture") {
        const bool pose = name == "set_pose";
        std::string line = pose ? "POSE" : "GESTURE";
        if (a["robot"].isString() && !a["robot"].asString().empty()) line += " " + quote(a["robot"].asString());
        if (pose) {
            bool any = false;
            if (a["posture"].isString()) line += " " + a["posture"].asString(), any = true;
            if (a["joints"].isObject())
                for (const auto& [k, v] : a["joints"].members()) {
                    if (!v.isNumber()) return textResult("joint '" + k + "' needs an angle in degrees", true);
                    line += " " + quote(k) + " " + num(v.asNumber());
                    any = true;
                }
            if (!any) return textResult("Give a 'posture' and/or 'joints'", true);
            if (a["seconds"].isNumber() && a["seconds"].asNumber() > 0) line += " IN " + num(a["seconds"].asNumber());
        } else {
            if (!a["gesture"].isString()) return textResult("'gesture' is required", true);
            line += " " + a["gesture"].asString();
            if (a["loop"].isBool()) line += a["loop"].asBool() ? " LOOP" : " ONCE";
            if (a["speed"].isNumber()) line += " SPEED " + num(a["speed"].asNumber());
            if (a["mirror"].asBool()) line += " MIRROR";
        }
        return runLine(line);
    }
    if (name == "physics_report") {
        std::string line = "PHYSICS AT " + num(a["time"].asNumber(0));
        if (a["body"].isString() && !a["body"].asString().empty()) line += " " + a["body"].asString();
        return runLine(line);
    }
    if (name == "set_physics") {
        std::string log;
        bool ok = true;
        auto step = [&](const std::string& line) {
            const Json r = runLine(line);
            log += (log.empty() ? "" : "\n") + r["content"][0]["text"].asString();
            return !r["isError"].asBool();
        };
        auto vec = [&](const Json& v) { return num(v[0].asNumber()) + "," + num(v[1].asNumber()) + "," + num(v[2].asNumber()); };
        if (a["demo"].isString()) ok = step("PHYSICS DEMO " + a["demo"].asString()) && ok;
        if (a["enabled"].isBool()) ok = step(a["enabled"].asBool() ? "PHYSICS ON" : "PHYSICS OFF") && ok;
        if (a["gravity"].isString() || a["g"].isArray() || a["big_g"].isNumber()) {
            std::string line = "PHYSICS GRAVITY " +
                               (a["gravity"].isString() ? a["gravity"].asString() : std::string(gravityModeKey(m_doc.scene.physics().gravity)));
            if (a["g"].isArray()) line += " " + vec(a["g"]);
            if (a["big_g"].isNumber()) line += " G " + num(a["big_g"].asNumber());
            ok = step(line) && ok;
        }
        if (a["floor"].isBool() || a["friction"].isNumber()) {
            std::string line = "PHYSICS FLOOR";
            line += a["floor"].isBool() ? (a["floor"].asBool() ? " ON" : " OFF") : (m_doc.scene.physics().floor ? " ON" : " OFF");
            if (a["friction"].isNumber()) line += " FRICTION " + num(a["friction"].asNumber());
            ok = step(line) && ok;
        }
        if (a["air"].isBool()) ok = step(a["air"].asBool() ? "PHYSICS AIR ON" : "PHYSICS AIR OFF") && ok;
        if (a["electric_field"].isArray()) ok = step("PHYSICS FIELD E " + vec(a["electric_field"])) && ok;
        if (a["magnetic_field"].isArray()) ok = step("PHYSICS FIELD B " + vec(a["magnetic_field"])) && ok;
        if (a["ambient_c"].isNumber()) ok = step("PHYSICS AMBIENT " + num(a["ambient_c"].asNumber())) && ok;
        if (a["relativity"].isBool() || a["light_speed"].isNumber()) {
            std::string line = "PHYSICS RELATIVITY";
            line += a["relativity"].isBool() ? (a["relativity"].asBool() ? " ON" : " OFF") : (m_doc.scene.physics().relativity ? " ON" : " OFF");
            if (a["light_speed"].isNumber()) line += " C " + num(a["light_speed"].asNumber());
            ok = step(line) && ok;
        }
        if (a["duration"].isNumber()) ok = step("PHYSICS DURATION " + num(a["duration"].asNumber())) && ok;
        if (const Json& b = a["body"]; b.isObject()) {
            if (!b["name"].isString()) return textResult("body needs a 'name'", true);
            std::string line = "PHYSICS BODY " + quote(b["name"].asString());
            const std::string mode = b["mode"].asString("dynamic");
            line += mode == "fixed" ? " FIXED" : mode == "off" ? " OFF" : " DYNAMIC";
            if (b["mass"].isNumber()) line += " MASS " + num(b["mass"].asNumber());
            if (b["density"].isNumber()) line += " DENSITY " + num(b["density"].asNumber());
            if (b["velocity"].isArray()) line += " VELOCITY " + vec(b["velocity"]);
            if (b["spin"].isArray()) line += " SPIN " + vec(b["spin"]);
            if (b["charge"].isNumber()) line += " CHARGE " + num(b["charge"].asNumber());
            if (b["temperature_c"].isNumber()) line += " TEMP " + num(b["temperature_c"].asNumber());
            if (b["specific_heat"].isNumber()) line += " HEAT " + num(b["specific_heat"].asNumber());
            if (b["restitution"].isNumber()) line += " BOUNCE " + num(b["restitution"].asNumber());
            if (b["force"].isArray()) line += " FORCE " + vec(b["force"]);
            if (b["wave"].isString()) line += " WAVE " + b["wave"].asString();
            if (b["wave_frequency"].isNumber()) line += " FREQ " + num(b["wave_frequency"].asNumber());
            if (b["wave_speed"].isNumber()) line += " WAVESPEED " + num(b["wave_speed"].asNumber());
            ok = step(line) && ok;
        }
        if (log.empty()) return textResult("Nothing to change", true);
        return textResult(log, !ok);
    }
    if (name == "list_parts") return runLine("PART");
    if (name == "insert_part") {
        if (!a["part"].isString()) return textResult("'part' is required (see list_parts)", true);
        std::string line = "PART " + a["part"].asString();
        if (a["name"].isString() && !a["name"].asString().empty()) line += " NAME " + quote(a["name"].asString());
        if (a["position"].isArray())
            line += " @" + num(a["position"][0].asNumber()) + "," + num(a["position"][1].asNumber()) + "," +
                    num(a["position"][2].asNumber());
        if (a["params"].isObject())
            for (const auto& [k, v] : a["params"].members()) {
                if (!v.isNumber()) return textResult("parameter '" + k + "' must be a number", true);
                line += " " + k + " " + num(v.asNumber());
            }
        return runLine(line);
    }
    if (name == "create_motion") {
        const std::string kind = a["kind"].asString();
        if (kind != "circular" && kind != "linear") return textResult("kind must be circular or linear", true);
        std::string line = kind == "circular" ? "MOTION CIRCULAR" : "MOTION LINEAR";
        if (a["name"].isString() && !a["name"].asString().empty()) line += " NAME " + quote(a["name"].asString());
        auto vec = [&](const char* key) {
            const Json& v = a[key];
            return num(v[0].asNumber()) + "," + num(v[1].asNumber()) + "," + num(v[2].asNumber());
        };
        if (a["axis"].isArray()) line += " AXIS " + vec("axis");
        if (a["origin"].isArray()) line += " AT " + vec("origin");
        if (a["each_centre"].isBool()) line += a["each_centre"].asBool() ? " EACH" : " SHARED";
        if (a["speed"].isNumber()) line += " SPEED " + num(a["speed"].asNumber());
        if (a["rpm"].isNumber()) line += " RPM " + num(a["rpm"].asNumber());
        if (a["range"].isNumber()) line += " RANGE " + num(a["range"].asNumber());
        if (a["mode"].isString()) line += " MODE " + a["mode"].asString();
        if (a["start"].isNumber()) line += " START " + num(a["start"].asNumber());
        if (a["follow"].isString()) line += " FOLLOW " + quote(a["follow"].asString());
        if (a["ratio"].isNumber()) line += " RATIO " + num(a["ratio"].asNumber());
        if (a["radius"].isNumber()) line += " RADIUS " + num(a["radius"].asNumber());
        if (a["along_loop"].asBool()) line += " PATH";
        if (a["follow"].isString())
            line += " FOLLOW " + (a["follow"].asString().empty() ? std::string("NONE") : quote(a["follow"].asString()));
        if (a["radius"].isNumber()) line += " RADIUS " + num(a["radius"].asNumber());
        if (a["bodies"].isArray() && a["bodies"].size() > 0) {
            line += " BODIES";
            for (const Json& b : a["bodies"].items()) line += " " + quote(b.asString() == "*" ? "ALL" : b.asString());
        }
        Json r = runLine(line);
        if (!r["isError"].asBool() && a["enabled"].isBool() && !m_doc.scene.motions().empty()) {
            // The motion just added or updated is the one with that name (or the newest).
            std::string target = m_doc.scene.motions().back().name;
            if (a["name"].isString())
                for (const Motion& m : m_doc.scene.motions())
                    if (m.name == a["name"].asString()) target = m.name;
            runLine(std::string(a["enabled"].asBool() ? "MOTION ON " : "MOTION OFF ") + target);
        }
        return r;
    }
    if (name == "delete_motion") return runLine("MOTION DELETE " + a["name"].asString());
    if (name == "simulate") {
        const std::string act = a["action"].asString();
        const std::string t = num(a["time"].asNumber());
        const std::string v = num(a["value"].asNumber());
        if (act == "poses") return runLine("SIM AT " + t);
        if (act == "seek") return runLine("SIM TIME " + t);
        if (act == "speed" || act == "duration") return runLine("SIM " + toUpper(act) + " " + v);
        if (act == "axes" || act == "paths")
            return runLine("SIM " + toUpper(act) + (a["value"].isNumber() ? (a["value"].asNumber() != 0 ? " ON" : " OFF") : ""));
        if (act == "open" || act == "play" || act == "pause" || act == "stop") return runLine("SIM " + toUpper(act));
        return textResult("unknown action", true);
    }
    if (name == "save_project")
        return runLine(a["path"].isString() ? "SAVE " + quote(a["path"].asString()) : std::string("SAVE"));
    if (name == "open_project") return runLine("OPEN " + quote(a["path"].asString()));
    if (name == "new_project") return runLine("NEW");
    if (name == "import_file") {
        std::string line = "IMPORT " + quote(a["path"].asString());
        if (a["position"].isArray()) line += " @" + pointArg(a["position"]);
        if (a["width"].isNumber()) line += " WIDTH " + num(a["width"].asNumber());
        return runLine(line);
    }
    if (name == "export_file") return runLine("EXPORT " + quote(a["path"].asString()));
    return Json(); // unknown tool: caller reports a protocol error
}

Json McpServer::handleJson(const Json& msg) {
    if (!msg.isObject() || msg["jsonrpc"].asString() != "2.0" || !msg["method"].isString())
        return errorReply(msg["id"], -32600, "Invalid Request");
    const std::string method = msg["method"].asString();
    const bool isNotification = !msg.has("id");
    const Json& id = msg["id"];
    const Json& params = msg["params"];

    if (isNotification) return Json(); // notifications/initialized, cancelled, ...

    if (method == "initialize") {
        std::string version = kSupportedVersions[0];
        const std::string requested = params["protocolVersion"].asString();
        for (const char* v : kSupportedVersions)
            if (requested == v) version = v;
        Json caps = Json::object();
        Json toolsCap = Json::object();
        toolsCap.set("listChanged", false);
        caps.set("tools", toolsCap);
        Json info = Json::object();
        info.set("name", "si-cdm").set("title", "SI CDM CAD/CAM").set("version", kServerVersion);
        Json r = Json::object();
        r.set("protocolVersion", version).set("capabilities", caps).set("serverInfo", info);
        r.set("instructions", kInstructions);
        m_initialized = true;
        return resultReply(id, r);
    }
    if (method == "ping") return resultReply(id, Json::object());
    if (method == "tools/list") {
        Json r = Json::object();
        r.set("tools", toolDefinitions());
        return resultReply(id, r);
    }
    if (method == "tools/call") {
        const std::string name = params["name"].asString();
        const Json& args = params["arguments"].isObject() ? params["arguments"] : Json::object();
        // Dedicated tools speak millimetres; run_command uses the document's units.
        const bool api = name != "run_command" && name != "list_commands";
        m_doc.apiMillimeters = api;
        Json result = callTool(name, args);
        m_doc.apiMillimeters = false;
        if (result.isNull()) return errorReply(id, -32602, "Unknown tool: " + name);
        return resultReply(id, result);
    }
    return errorReply(id, -32601, "Method not found: " + method);
}

std::string McpServer::handle(std::string_view message) {
    std::string err;
    const auto msg = Json::parse(message, &err);
    if (!msg) return errorReply(Json(), -32700, "Parse error: " + err).dump();
    const Json reply = handleJson(*msg);
    return reply.isNull() ? std::string() : reply.dump();
}

} // namespace sicdm

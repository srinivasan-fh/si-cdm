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

// A quoted command argument; quotes inside become \".
std::string quote(const std::string& s) {
    std::string q = "\"";
    for (const char c : s) {
        if (c == '"') q += '\\';
        q += c;
    }
    return q + "\"";
}

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
    case CurveKind::Arc: return "arc";
    case CurveKind::Spline: return "spline";
    case CurveKind::Ellipse: return "ellipse";
    case CurveKind::Slot: return "slot";
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
        p.set("kind", enumProp({"point", "line", "polyline", "rectangle", "circle", "arc", "spline", "ellipse", "slot", "polygon"}, "Curve type"));
        Json pts = prop("array", "2D points [[u,v], ...] in plane mm. line: 2 or more (connected segments); "
                                 "rectangle: 2 opposite corners; circle, polygon: centre; point: 1; arc: start, a point on it, end; "
                                 "spline: the points it passes through; ellipse: centre, major axis end; slot: the centres of its ends");
        Json pair = Json::object();
        pair.set("type", "array");
        Json num1 = Json::object();
        num1.set("type", "number");
        pair.set("items", num1).set("minItems", 2).set("maxItems", 2);
        pts.set("items", pair);
        p.set("points", pts);
        p.set("radius", prop("number", "Circle radius; ellipse minor radius; slot half width; polygon radius to the corners (mm)"));
        p.set("sides", prop("number", "Polygon: number of sides (default 6)"));
        p.set("construction", prop("boolean", "A construction (reference) curve: dashed, not part of profiles"));
        p.set("plane", enumProp({"top", "front", "right"}, "Sketch plane (default: active plane)"));
        p.set("closed", prop("boolean", "Polyline, spline: close back to the first point"));
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
        p.set("operation", prop("string",
                                "A brush: draw (pull; invert pushes), clay, claystrips, layer, carve, inflate, blob, grab, "
                                "snakehook, nudge, pinch, crease, smooth, flatten, scrape, fill, mask -- or hole, pocket, "
                                "channel, trim (knife), boss (add volume), boolean, refine, remesh (voxel: fuse 'body' with "
                                "'others'), subdivide, symmetrize, decimate, mask_clear, mask_invert, mask_smooth, undo, redo"));
        p.set("body", prop("string", "Body to work on (default: the selection)"));
        p.set("points", prop("array", "Points [[x,y,z], ...] in mm on the surface: the stroke, the hole / pocket / boss "
                                      "position, the channel path or a point of the knife plane"));
        p.set("direction", prop("array", "[x,y,z] tool direction into the body (default: against the surface normal)"));
        p.set("normal", prop("array", "trim: [x,y,z] normal of the knife plane; the side it points to is removed"));
        p.set("smaller", prop("boolean", "trim: remove whichever side is smaller"));
        p.set("radius", prop("number", "Brush radius, mm (default 20)"));
        p.set("strength", prop("number", "Brush strength 0-1 (default 0.5)"));
        p.set("invert", prop("boolean", "Invert the brush (push instead of pull, remove instead of add)"));
        p.set("symmetry", prop("boolean", "Mirror the stroke across the body's centre (X)"));
        p.set("symmetry_axes", prop("string", "Mirror across these of the body's centre planes: any of \"xyz\""));
        p.set("radial", prop("number", "Radial symmetry: copies around the body's axis (radial_axis, default z)"));
        p.set("radial_axis", prop("string", "x | y | z"));
        p.set("falloff", prop("string", "smooth | sphere | sharp | linear | constant"));
        p.set("levels", prop("number", "subdivide: levels (default 1)"));
        p.set("axis", prop("string", "symmetrize: x | y | z, with a leading - to keep the negative side"));
        p.set("target", prop("number", "decimate: triangles to keep"));
        p.set("detail", prop("number", "Longest edge left under the brush, mm (refine: the target edge length)"));
        p.set("move", prop("array", "grab: [dx,dy,dz] mm"));
        p.set("diameter", prop("number", "hole / boss diameter, mm"));
        p.set("depth", prop("number", "hole / pocket / channel depth, mm (hole: omit for through)"));
        p.set("width", prop("number", "pocket / channel width, mm"));
        p.set("length", prop("number", "pocket length, mm"));
        p.set("height", prop("number", "boss height, mm"));
        p.set("angle", prop("number", "pocket rotation about the surface normal, degrees"));
        p.set("shape", prop("string", "pocket: box|round|ball; boss: cylinder|box|ball"));
        p.set("other", prop("string", "boolean: the second body"));
        p.set("boolean_op", prop("string", "boolean: union|subtract|intersect"));
        p.set("keep", prop("boolean", "boolean: keep the second body (default: it is used up)"));
        p.set("others", prop("array", "remesh: names of bodies fused into 'body' (they are used up)"));
        p.set("voxel", prop("number", "remesh: voxel size, mm (default: the size / 160)"));
        p.set("blend", prop("number", "remesh: fillet radius where parts meet, mm"));
        p.set("smooth", prop("number", "remesh: rounds over detail finer than this, mm"));
        tools.push(tool("sculpt",
                        "Digital sculpting and CAD cuts on a body's mesh: clay brushes with dynamic topology, exact "
                        "boolean holes, pockets, channels, knife trims, bosses and body unions / subtractions / "
                        "intersections. Results stay closed meshes, ready for STL / OBJ export (3D printing, CAM).",
                        p, {"operation"}));
    }
    {
        Json p = Json::object();
        p.set("operation", prop("string", "shell | armor | joint | lighten | interference"));
        p.set("body", prop("string", "Body (shell, lighten, joint: the moving part; armor: a body or, with part, the robot)"));
        p.set("part", prop("string", "armor: armour every part of the robot whose name contains this (e.g. forearm); "
                                     "several words separated by commas"));
        p.set("merge", prop("boolean", "armor: one shell around all the matched parts (a chest plate over chest and shoulders)"));
        p.set("thickness", prop("number", "shell / armour wall, mm"));
        p.set("clearance", prop("number", "armour gap to the body, or the interference clearance, mm"));
        p.set("outward", prop("boolean", "shell: grow outwards"));
        p.set("open_point", prop("array", "shell: [x,y,z] on the opening plane"));
        p.set("open_normal", prop("array", "shell: [x,y,z]; the side it points to is cut open"));
        p.set("parent", prop("string", "joint: the part it hinges on"));
        p.set("at", prop("array", "joint: [x,y,z] pivot, mm"));
        p.set("axis", prop("array", "joint: [x,y,z] hinge axis"));
        p.set("ball", prop("boolean", "joint: a ball joint instead of a hinge"));
        p.set("range", prop("array", "joint: [lo, hi] degrees"));
        p.set("period", prop("number", "joint: seconds per swing"));
        p.set("skin", prop("number", "lighten: solid skin, mm"));
        p.set("cell", prop("number", "lighten: gyroid cell, mm"));
        p.set("wall", prop("number", "lighten: lattice wall, mm"));
        p.set("hollow", prop("boolean", "lighten: hollow only, no lattice"));
        p.set("step", prop("number", "interference: seconds between samples"));
        p.set("duration", prop("number", "interference: seconds of the timeline to sweep"));
        p.set("bodies", prop("array", "interference: names of the bodies to check against each other"));
        tools.push(tool("armor",
                        "Wearable armour and exoskeletons over a human base: hollow / solidify shells (with an opening), "
                        "armour offset around body parts with a clearance gap (moves with the part), hinge and ball "
                        "joints, gyroid-lattice lightweighting, and an interference check over the motions.",
                        p, {"operation"}));
    }
    {
        Json p = Json::object();
        p.set("operation", prop("string", "check | repair | orient | slice | export_3mf"));
        p.set("body", prop("string", "Body (default: the selection)"));
        p.set("path", prop("string", "slice: the .gcode file; export_3mf: the .3mf file"));
        p.set("overhang", prop("number", "Degrees from vertical a face may lean out before it needs support (default 45)"));
        p.set("min_wall", prop("number", "check: thinnest printable wall, mm (default 0.8)"));
        p.set("layer", prop("number", "slice: layer height, mm (default 0.2)"));
        p.set("width", prop("number", "slice: line width, mm (default 0.45)"));
        p.set("infill", prop("number", "slice: infill percent (default 20)"));
        p.set("walls", prop("number", "slice: wall lines (default 2)"));
        p.set("nozzle", prop("number", "slice: nozzle temperature, deg C (default 210)"));
        p.set("bed", prop("number", "slice: bed temperature, deg C (default 60)"));
        tools.push(tool("print",
                        "3D-print preparation: a printability check (holes, non-manifold edges, inside-out faces, "
                        "overhangs needing support, thin walls, size, material and time), mesh repair, orientation for "
                        "the least support, 3MF export, and an FDM slicer writing G-code.",
                        p, {"operation"}));
    }
    {
        Json p = Json::object();
        p.set("operation", prop("string", "setup | tool | library | face | rough | pocket | contour | parallel | waterline | drill | bore | rotary | "
                                          "5axis | machine | holes | list | generate | simulate | post | delete | clear"));
        p.set("library", prop("string", "library: list | import | export | store | use | delete | load | save - the user's tool library: import a "
                                        "vendor catalogue (.csv) or Fusion 360 library (.json) into it, use a library tool in the job, store job "
                                        "tools in it, or load / save the job's tools straight from / to a file"));
        p.set("entry", prop("string", "library use / delete: the library tool - its number in the listing (#3), product id or name; library "
                                      "store: a job tool number, or ALL; library list: text to search for"));
        p.set("as", prop("number", "library use: the tool number in the job (default the library's, or the next free)"));
        p.set("file_format", prop("string", "library export / save: csv or fusion (default: by the extension, else the native JSON)"));
        p.set("body", prop("string", "setup: the part (default: the selection)"));
        p.set("stock_margin", prop("number", "setup: stock around the part, mm (default 2; 1 on top)"));
        p.set("stock_size", prop("array", "setup: stock size [x, y, z], mm, centred on the part, from its bottom"));
        p.set("material", prop("string", "setup: aluminium, steel, stainless, brass, castiron, titanium, plastic, wood"));
        p.set("origin", prop("string", "setup: work zero at the stock's top 'corner' (default), 'centre', or the 'model' origin"));
        p.set("post", prop("string", "setup / post: fanuc, haas, grbl, linuxcnc"));
        p.set("preset", prop("string", "machine: the preset - generic, vmc, vmc-4th (A: 4-axis), trunnion (A and C: 3+2 and 5-axis), benchtop, router"));
        p.set("number", prop("number", "tool: T number"));
        p.set("type", prop("string", "tool: flat, ball, bull, face, drill, spot, chamfer, tap"));
        p.set("diameter", prop("number", "tool: diameter, mm; drill/bore with points: the hole size"));
        p.set("corner", prop("number", "tool: bull-nose corner radius, mm"));
        p.set("flutes", prop("number", "tool: flutes"));
        p.set("length", prop("number", "tool: flute length, mm"));
        p.set("stickout", prop("number", "tool: length out of the holder, mm (checked in the simulation)"));
        p.set("holder", prop("number", "tool: the holder's diameter, mm"));
        p.set("tool", prop("number", "operations: the T number (default: the first that suits)"));
        p.set("stepdown", prop("number", "operations: depth of cut, mm"));
        p.set("stepover", prop("number", "operations: width of cut, mm"));
        p.set("stepover_percent", prop("number", "operations: width of cut, percent of the tool's diameter"));
        p.set("leave", prop("number", "operations: stock to leave, mm"));
        p.set("top", prop("number", "operations: top z, mm"));
        p.set("bottom", prop("number", "operations: bottom z, mm"));
        p.set("depth", prop("number", "operations: depth below the stock top (drill: below the hole's top), mm"));
        p.set("side", prop("string", "contour: outside (default), inside, both"));
        p.set("tabs", prop("number", "contour: holding tabs on the outside profile"));
        p.set("scallop", prop("number", "parallel / waterline / 5axis: cusp height, mm"));
        p.set("slope", prop("array", "parallel / waterline / 5axis: [from, to] degrees of surface slope to cut"));
        p.set("max_tilt", prop("number", "5axis: the most the tool leans from vertical, degrees (default 60)"));
        p.set("lead_angle", prop("number", "5axis: the tool leaning forward along the cut from the surface normal, degrees (default 10)"));
        p.set("side_tilt", prop("number", "5axis: the tool leaning sideways across the cut, degrees (default 0)"));
        p.set("raster", prop("boolean", "rough / pocket: zig-zag instead of offset rings"));
        p.set("adaptive", prop("boolean", "rough / pocket: adaptive clearing - light stepover, deep cuts, trochoidal loops where the tool would be overloaded"));
        p.set("index_a", prop("number", "any 3-axis operation: 3+2, the table turned to A degrees (about X) first"));
        p.set("index_c", prop("number", "any 3-axis operation: 3+2, the table turned to C degrees (about Z, before A) first"));
        p.set("along", prop("boolean", "rotary: passes along the axis instead of a helix round it"));
        p.set("rest", prop("boolean", "rough / pocket: only the material the operations before left (rest machining)"));
        p.set("cycle", prop("string", "drill: spot, drill, peck, chipbreak, tap, bore"));
        p.set("peck", prop("number", "drill: peck depth, mm"));
        p.set("points", prop("array", "drill / bore: hole centres [[x, y], ...] (default: the part's holes that fit the tool)"));
        p.set("path", prop("string", "post: the G-code file; library import / export / load / save: the tool file"));
        p.set("show", prop("boolean", "simulate: add the cut stock as a body"));
        p.set("index", prop("number", "delete: the operation's number (1-based)"));
        p.set("name", prop("string", "tool / operations: a name"));
        tools.push(tool("cam",
                        "CNC milling CAM: set up the part and stock, build a tool library with feeds and speeds, add operations "
                        "(facing, 2D contour with tabs, pocket, 3D roughing - adaptive or rest - parallel and waterline finishing, drilling "
                        "cycles, helical boring, 3+2 indexed versions of any of them, 4-axis rotary wrapping, and simultaneous 5-axis surface finishing with a ball end mill on a table-table machine), simulate the cut "
                        "(stock left, gouges, rapid crashes, holder collisions, cycle time; voxels when the part turns) and post G-code for "
                        "Fanuc, Haas, GRBL or LinuxCNC.",
                        p, {"operation"}));
    }
    {
        Json p = Json::object();
        p.set("operation", prop("string", "setup | tool | tools | profile | face | rough | finish | bore | groove | drill | thread | partoff | list | "
                                          "generate | simulate | post | delete | clear"));
        p.set("body", prop("string", "setup: the part (default: the selection)"));
        p.set("axis", prop("string", "setup: the model's axis the part turns about, x, y or z (default: its longest side)"));
        p.set("flip", prop("boolean", "setup: face the part's -axis end instead of its +axis end"));
        p.set("bar", prop("number", "setup: the bar's diameter, mm (default: the part's plus 2, to a whole mm)"));
        p.set("front", prop("number", "setup: bar in front of the part to face off, mm (default 1)"));
        p.set("material", prop("string", "setup: aluminium, steel, stainless, brass, castiron, titanium, plastic, wood"));
        p.set("post", prop("string", "setup / post: fanuc, haas, linuxcnc"));
        p.set("max_rpm", prop("number", "setup: the spindle's limit (constant surface speed is clamped to it)"));
        p.set("number", prop("number", "tool: T number"));
        p.set("type", prop("string", "tool: turn, bore (boring bar), groove (grooving / parting blade), thread, drill"));
        p.set("nose", prop("number", "tool: the insert's nose radius, mm (a blade's corners)"));
        p.set("insert", prop("string", "tool: turning / boring insert shape C or W (80 deg), D (55), V (35), T (60), S (90), or an angle"));
        p.set("size", prop("number", "tool: a drill's diameter, a boring bar's smallest bore, a blade's width, mm"));
        p.set("reach", prop("number", "tool: how deep a blade, drill or boring bar goes, mm"));
        p.set("tool", prop("number", "operations: the T number (default: the first that suits)"));
        p.set("inside", prop("boolean", "rough / finish / groove / thread: the bore instead of the outside"));
        p.set("stepdown", prop("number", "face / rough: depth of cut, mm"));
        p.set("leave", prop("number", "rough / finish: stock to leave radially, mm"));
        p.set("leave_z", prop("number", "rough / finish: stock to leave on faces, mm"));
        p.set("from", prop("number", "operations: start Z (0 is the part's front face, negative towards the chuck)"));
        p.set("to", prop("number", "operations: end Z"));
        p.set("depth", prop("number", "drill: depth from the front face, mm (default: the part's bore)"));
        p.set("peck", prop("number", "drill / partoff: peck depth, mm"));
        p.set("pitch", prop("number", "thread: the pitch, mm"));
        p.set("diameter", prop("number", "thread: the major diameter, mm (default: the part's there)"));
        p.set("speed", prop("number", "operations: cutting speed, m/min"));
        p.set("feed", prop("number", "operations: feed, mm per turn"));
        p.set("rpm", prop("number", "operations: a constant spindle speed instead of constant surface speed"));
        p.set("path", prop("string", "post: the G-code file"));
        p.set("show", prop("boolean", "simulate: add the turned stock as a body"));
        p.set("index", prop("number", "delete: the operation's number (1-based)"));
        p.set("name", prop("string", "tool / operations: a name"));
        tools.push(tool("lathe",
                        "CNC turning: set up a part to turn about an axis from bar stock, add turning inserts, boring bars, grooving / "
                        "parting blades, drills and threading inserts, then facing, rough and finish turning and boring (stock to leave, "
                        "the insert's angle limiting how steeply it goes down behind shoulders), grooving the recesses a turning insert "
                        "can't reach, drilling on the axis, single-point threading and parting off; simulate the turned profile (stock "
                        "left, gouges, rapid crashes, cycle time at constant surface speed) and post Fanuc, Haas or LinuxCNC lathe G-code.",
                        p, {"operation"}));
    }
    {
        Json p = Json::object();
        p.set("operation", prop("string", "revolve | sweep | loft | pattern | mirror | edges | fillet | chamfer | massprops | measure | history | regen | "
                                          "history_edit | history_insert | history_delete | constrain | dim | constraints | "
                                          "constraint_edit | constraint_delete | drag"));
        p.set("kind", prop("string", "constrain: coincident, horizontal, vertical, parallel, perpendicular, tangent, equal, concentric, on, "
                                     "midpoint, fix; dim: distance (default), horizontal, vertical, radius, diameter, angle"));
        p.set("refs", prop("array", "constrain / dim: the curves, each {\"curve\": name, \"at\": \"start\" | \"end\" | \"centre\" | "
                                    "\"point n\" | \"side n\"}"));
        p.set("value", prop("number", "dim / constraint_edit: the size (mm, or degrees for an angle); dim without it keeps the size now"));
        p.set("constraint", prop("string", "constraint_edit / delete: its name (D1, C2...)"));
        p.set("grip", prop("string", "drag: what of 'curve' to drag - \"start\", \"end\", \"centre\", \"point n\", \"rim\" (a circle's or "
                                     "arc's radius); omitted: the whole curve"));
        p.set("to", prop("array", "drag: [x, y] mm on the sketch plane where the grip goes (a whole curve: where 'from' goes)"));
        p.set("from", prop("array", "drag a whole curve: [x, y] mm, the point grabbed (default its first point)"));
        p.set("by", prop("array", "drag a whole curve: [dx, dy] mm to move it by"));
        p.set("curve", prop("string", "revolve: the closed sketch curve; sweep: the profile"));
        p.set("path", prop("string", "sweep: the path curve"));
        p.set("sections", prop("array", "loft: the closed sketch curves, in order"));
        p.set("offsets", prop("array", "loft: each section's offset along its plane's normal, mm"));
        p.set("ruled", prop("boolean", "loft: straight between sections (default smooth)"));
        p.set("angle", prop("number", "revolve / circular pattern: degrees (default 360)"));
        p.set("axis", prop("string", "revolve: u, v or a sketch line's name; pattern: x, y or z"));
        p.set("body", prop("string", "pattern / mirror / edges / fillet / chamfer / massprops / measure: the body (default: the selection)"));
        p.set("type", prop("string", "pattern: linear or circular"));
        p.set("count", prop("number", "pattern: copies including the original"));
        p.set("spacing", prop("number", "linear pattern: mm between copies"));
        p.set("direction", prop("array", "linear pattern: [x, y, z]"));
        p.set("rows", prop("number", "linear pattern: rows in a second direction"));
        p.set("spacing2", prop("number", "linear pattern: mm between rows"));
        p.set("direction2", prop("array", "linear pattern: the rows' direction [x, y, z]"));
        p.set("center", prop("array", "circular pattern: a point on the axis [x, y, z]; mirror: a point on the plane"));
        p.set("union", prop("string", "pattern: join the copies into this body"));
        p.set("subtract", prop("string", "pattern: cut the copies from this body"));
        p.set("plane", prop("string", "mirror: xy, yz or xz (default yz)"));
        p.set("normal", prop("array", "mirror: the plane's normal [x, y, z]"));
        p.set("move", prop("boolean", "mirror: flip the body itself instead of making a copy"));
        p.set("join", prop("boolean", "mirror: join the copy to the body"));
        p.set("radius", prop("number", "fillet: the ball's radius, mm"));
        p.set("edges", prop("string", "fillet without 'select': round every convex (default), concave or both edge with a rolling ball"));
        p.set("select", prop("string", "fillet / chamfer: the edges to treat exactly - numbers from 'edges' (\"1 3 4\"), or all, convex, "
                                       "concave, vertical, horizontal, top, bottom, or \"near x,y,z\""));
        p.set("distance", prop("number", "chamfer: the setback along each face, mm"));
        p.set("material", prop("string", "massprops: steel, aluminium, brass, copper, titanium, abs, pla, wood, ..."));
        p.set("density", prop("number", "massprops: g/cm3"));
        p.set("other", prop("string", "measure: the second body"));
        p.set("points", prop("array", "measure: two points [[x, y, z], [x, y, z]]"));
        p.set("step", prop("number", "history_edit / insert / delete: the step's number (1-based)"));
        p.set("command", prop("string", "history_edit / insert: the new command line"));
        p.set("force", prop("boolean", "regen: rebuild even if the model was changed outside commands"));
        tools.push(tool("cad",
                        "Part features: revolve, sweep and loft sketch curves into solids; linear and circular patterns; mirror; "
                        "fillets and chamfers on chosen edges (or rolling-ball on all); mass properties (volume, mass, centre of mass, inertia); measure distances; and "
                        "the feature history (list, edit, insert, delete a step, regenerate); sketch constraints and dimensions, and dragging "
                        "sketch geometry with them held (as with the mouse).",
                        p, {"operation"}));
    }
    {
        Json p = Json::object();
        p.set("operation", prop("string", "mate | fix | list | solve | edit | flip | suppress | unsuppress | delete | clear | tree | group | ungroup | "
                                          "interference | bom"));
        p.set("type", prop("string", "mate: coincident, flush, concentric, distance, angle, parallel or perpendicular"));
        p.set("part", prop("string", "mate: the part that stays; fix: the part to hold still"));
        p.set("at", prop("array", "mate: a point [x, y, z] on the face of `part` to mate (mm, world)"));
        p.set("moving", prop("string", "mate: the part that moves"));
        p.set("moving_at", prop("array", "mate: a point [x, y, z] on the face of `moving`"));
        p.set("value", prop("number", "mate distance (mm) or angle (degrees); edit: the new value"));
        p.set("flip", prop("boolean", "mate: planes the other way round (flush instead of facing)"));
        p.set("mate", prop("string", "edit / flip / suppress / unsuppress / delete: the mate's name (M1...) or number"));
        p.set("name", prop("string", "group: the sub-assembly (a/b for one inside another)"));
        p.set("parts", prop("array", "group / ungroup / interference: part names"));
        tools.push(tool("assembly",
                        "Assemblies: mate parts by their faces (coincident, flush, concentric, distance, angle, parallel, perpendicular; "
                        "planes, axes of round faces and sphere centres picked by a point on the face) and the solver moves the second "
                        "part into place, reporting the freedom left and conflicts; fix parts; the tree of sub-assemblies (kept in STEP "
                        "files); interference between parts with overlap volumes; and the parts list.",
                        p, {"operation"}));
    }
    {
        Json p = Json::object();
        p.set("operation", prop("string", "new | list | view_add | view_move | view_delete | view_scale | view_hidden | section | dim | dim_delete | "
                                          "autodim | tolerance | gdt | gdt_delete | datum | datum_delete | finish | general_tolerance | title | note | "
                                          "scale | size | projection | export | delete"));
        p.set("bodies", prop("array", "new: the bodies to draw (default: every visible body)"));
        p.set("size", prop("string", "new / size: A4, A3, A2, A1, A0, LETTER, TABLOID, ANSI C, ANSI D (default A3)"));
        p.set("portrait", prop("boolean", "new / size: portrait instead of landscape"));
        p.set("third_angle", prop("boolean", "new / projection: third-angle (ASME) instead of first-angle (ISO)"));
        p.set("scale", prop("string", "new / scale / view_add / view_scale: 1:2, 2:1 ... (default: a standard scale that fits)"));
        p.set("view", prop("string", "view_*: the view's name; view_add: front, back, top, bottom, left, right or iso; section / dim / autodim: the view"));
        p.set("at", prop("array", "view_add / view_move / note: [x, y] on the sheet, mm from the bottom-left corner; section: the cut's position, model mm"));
        p.set("horizontal", prop("boolean", "section: cut across horizontally (default vertically)"));
        p.set("flip", prop("boolean", "section: look the other way"));
        p.set("hidden", prop("boolean", "view_add / view_hidden: show hidden lines dashed"));
        p.set("kind", prop("string", "dim: horizontal, vertical, aligned, diameter or radius"));
        p.set("from", prop("array", "dim: the first model point [x, y, z] (diameter / radius: the centre)"));
        p.set("to", prop("array", "dim: the second model point [x, y, z] (diameter / radius: on the circle)"));
        p.set("offset", prop("number", "dim: sheet mm from the feature (negative: below / left)"));
        p.set("index", prop("number", "dim_delete: the dimension's number (1-based)"));
        p.set("field", prop("string", "title: TITLE, PART, MATERIAL, DRAWN, CHECKED, DATE, REV, SHEET"));
        p.set("text", prop("string", "title / note / dim: the text"));
        p.set("path", prop("string", "export: the file (.pdf, .svg or .dxf)"));
        p.set("tolerance", prop("string", "dim / tolerance: the size tolerance - \"0.1\" (+/-), \"+0.1 -0.05\", \"LIMITS +0.1 -0.05\", "
                                          "\"FIT H7\" (ISO 286; \"FIT H7 VALUES\" shows the limits), \"BASIC\" or \"NONE\"; "
                                          "general_tolerance: \"ISO 2768-mK\" or \"NONE\""));
        p.set("symbol", prop("string", "gdt: straightness, flatness, circularity, cylindricity, profile-line, profile-surface, angularity, "
                                       "perpendicularity, parallelism, position, concentricity, symmetry, runout, total-runout"));
        p.set("value", prop("number", "gdt: the tolerance zone (mm); finish: Ra (micrometres)"));
        p.set("diameter", prop("boolean", "gdt: a cylindrical zone (the diameter sign)"));
        p.set("material", prop("string", "gdt: MMC or LMC"));
        p.set("datums", prop("array", "gdt: the datum references, primary first, e.g. [\"A\", \"B\"] or [\"A\", \"B(M)\"]"));
        p.set("letter", prop("string", "datum / datum_delete: the datum letter"));
        p.set("point", prop("array", "gdt / datum / finish: the model point [x, y, z] in 'view' the leader goes to"));
        p.set("sheet_offset", prop("array", "gdt / datum / finish: where the symbol sits from the point, [dx, dy] sheet mm"));
        p.set("machined", prop("boolean", "finish: material removal required (false: not allowed; omit: any process)"));
        tools.push(tool("drawing",
                        "2D engineering drawings of the model: front, top, right and isometric views laid out on an ISO or ANSI sheet "
                        "(first or third angle) with hidden lines dashed, section views hatched, centre marks, dimensions (automatic overall "
                        "sizes and hole diameters, or by hand) with size tolerances and ISO 286 fits, geometric tolerances (feature control frames, "
                        "datums), surface finish marks, the general tolerance, a title block and notes; exported to PDF, SVG or DXF.",
                        p, {"operation"}));
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
        p.set("path", prop("string", ".step/.stp exact solids (assemblies placed, units converted), .stl/.obj mesh, .dxf drawing, or image (.png .jpg .bmp .tga .gif ...)"));
        p.set("position", numberArray(2, "Image centre [u,v] on the sketch plane (mm)"));
        p.set("width", prop("number", "Image width (mm)"));
        tools.push(tool("import_file", "Import STEP or IGES solids (exact B-reps) or a mesh as bodies, a DXF onto the sketch plane, or a reference image.",
                        p, {"path"}));
        Json e = Json::object();
        e.set("path", prop("string", ".step or .igs (visible bodies: exact where they have exact geometry, else planar facets), .stl/.obj/.3mf (visible bodies) or .dxf/.svg (curves on the sketch plane)"));
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
        } else if (kind == "line" || kind == "polyline" || kind == "rectangle" || kind == "spline" || kind == "arc") {
            line = kind == "line" ? "LINE" : kind == "polyline" ? "PLINE" : kind == "rectangle" ? "RECT" : kind == "spline" ? "SPLINE" : "ARC";
            for (const Json& p : pts) line += " " + pointArg(p);
            if ((kind == "polyline" || kind == "spline") && a["closed"].asBool()) line += " CLOSE";
        } else if (kind == "ellipse") {
            if (!a["radius"].isNumber() || pts.size() < 2) return textResult("ellipse needs points [centre, major axis end] and 'radius' (the minor)", true);
            line = "ELLIPSE " + pointArg(pts[0]) + " " + pointArg(pts[1]) + " " + num(a["radius"].asNumber());
        } else if (kind == "slot") {
            if (!a["radius"].isNumber() || pts.size() < 2) return textResult("slot needs points [end centre, end centre] and 'radius'", true);
            line = "SKETCHSLOT " + pointArg(pts[0]) + " " + pointArg(pts[1]) + " " + num(a["radius"].asNumber());
        } else if (kind == "polygon") {
            if (!a["radius"].isNumber()) return textResult("polygon needs points [centre] and 'radius' (to its corners)", true);
            line = "POLYGON " + pointArg(pts[0]) + " " + num(a["radius"].asNumber()) + " " + num(a["sides"].isNumber() ? a["sides"].asNumber() : 6);
        } else {
            m_doc.state.plane = old;
            return textResult("kind must be point, line, polyline, rectangle, circle, arc, spline, ellipse, slot or polygon", true);
        }
        if (a["construction"].asBool()) line += " CONSTRUCTION";
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
        for (std::size_t bi = 0; bi < bodies.size(); ++bi)
            if (Body& b = bodies[bi]; b.name == target) {
                if (a["position"].isArray()) b.offset = v3("position");
                if (a["move_by"].isArray()) b.offset += v3("move_by");
                if (a["visible"].isBool()) b.visible = a["visible"].asBool();
                if (a["color"].isArray()) b.color = v3("color");
                if (a["rename"].isString() && !a["rename"].asString().empty()) {
                    const std::string old = b.name, now = a["rename"].asString();
                    m_doc.scene.renameBody(bi, now);
                    for (Mate& m : m_doc.state.assembly.mates) {
                        if (m.a.body == old) m.a.body = now;
                        if (m.b.body == old) m.b.body = now;
                    }
                }
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
    if (name == "print") {
        const std::string op = a["operation"].asString();
        const std::string body = a["body"].isString() ? quote(a["body"].asString()) : "";
        auto opt = [&](const char* key, const char* word) {
            return a[key].isNumber() ? std::string(" ") + word + " " + num(a[key].asNumber()) : std::string();
        };
        if (op == "check") return runLine("PRINTCHECK " + body + opt("overhang", "OVERHANG") + opt("min_wall", "WALL"));
        if (op == "repair") return runLine("REPAIR " + body);
        if (op == "orient") return runLine("ORIENT " + body + opt("overhang", "OVERHANG"));
        if (op == "slice") {
            if (!a["path"].isString()) return textResult("slice needs 'path' (a .gcode file)", true);
            return runLine("SLICE " + body + " TO " + quote(a["path"].asString()) + opt("layer", "LAYER") + opt("width", "WIDTH") +
                           opt("infill", "INFILL") + opt("walls", "WALLS") + opt("nozzle", "NOZZLE") + opt("bed", "BED"));
        }
        if (op == "export_3mf") {
            if (!a["path"].isString()) return textResult("export_3mf needs 'path' (a .3mf file)", true);
            return runLine("EXPORT " + quote(a["path"].asString()));
        }
        return textResult("operation is one of check, repair, orient, slice, export_3mf", true);
    }
    if (name == "cad") {
        const std::string op = a["operation"].asString();
        auto opt = [&](const char* key, const char* word) {
            return a[key].isNumber() ? std::string(" ") + word + " " + num(a[key].asNumber()) : std::string();
        };
        auto named = [&](const char* key) { return a[key].isString() ? " " + quote(a[key].asString()) : std::string(); };
        auto vec = [&](const char* key) {
            const Json& v = a[key];
            return v.isArray() && v.size() == 3 ? num(v[0].asNumber()) + "," + num(v[1].asNumber()) + "," + num(v[2].asNumber()) : std::string();
        };
        if (op == "revolve") {
            std::string line = "REVOLVE" + named("curve") + opt("angle", "ANGLE");
            if (a["axis"].isString()) {
                const std::string ax = a["axis"].asString();
                line += " AXIS " + (ax == "u" || ax == "v" || ax == "U" || ax == "V" ? ax : quote(ax));
            }
            return runLine(line);
        }
        if (op == "sweep") {
            if (!a["path"].isString()) return textResult("sweep needs 'path'", true);
            return runLine("SWEEP" + named("curve") + " ALONG " + quote(a["path"].asString()));
        }
        if (op == "loft") {
            if (!a["sections"].isArray() || a["sections"].size() < 2) return textResult("loft needs two 'sections' or more", true);
            std::string line = "LOFT";
            for (std::size_t k = 0; k < a["sections"].size(); ++k) {
                line += " " + quote(a["sections"][k].asString());
                if (a["offsets"].isArray() && k < a["offsets"].size()) line += " AT " + num(a["offsets"][k].asNumber());
            }
            return runLine(line + (a["ruled"].asBool() ? " RULED" : ""));
        }
        if (op == "pattern") {
            const std::string type = a["type"].asString("linear");
            if (!a["count"].isNumber()) return textResult("pattern needs 'count'", true);
            std::string line = "PATTERN" + named("body") + (type == "circular" ? " CIRCULAR " : " LINEAR ") + num(a["count"].asNumber());
            line += opt("spacing", "SPACING") + opt("rows", "ROWS") + opt("spacing2", "SPACING2") + opt("angle", "ANGLE");
            if (!vec("direction").empty()) line += " DIR " + vec("direction");
            if (!vec("direction2").empty()) line += " DIR2 " + vec("direction2");
            if (a["axis"].isString()) line += " AXIS " + a["axis"].asString();
            if (!vec("center").empty()) line += " AT " + vec("center");
            if (a["union"].isString()) line += " UNION " + quote(a["union"].asString());
            if (a["subtract"].isString()) line += " SUBTRACT " + quote(a["subtract"].asString());
            return runLine(line);
        }
        if (op == "mirror") {
            std::string line = "MIRROR" + named("body");
            if (a["plane"].isString()) line += " PLANE " + a["plane"].asString();
            if (!vec("center").empty()) line += " AT " + vec("center");
            if (!vec("normal").empty()) line += " NORMAL " + vec("normal");
            if (a["move"].asBool()) line += " MOVE";
            if (a["join"].asBool()) line += " UNION";
            return runLine(line);
        }
        if (op == "fillet") {
            if (!a["radius"].isNumber()) return textResult("fillet needs 'radius'", true);
            if (a["select"].isString()) return runLine("FILLET" + named("body") + opt("radius", "RADIUS") + " EDGES " + a["select"].asString());
            return runLine("FILLET" + named("body") + opt("radius", "RADIUS") + (a["edges"].isString() ? " " + toUpper(a["edges"].asString()) : std::string()));
        }
        if (op == "chamfer") {
            if (!a["distance"].isNumber()) return textResult("chamfer needs 'distance'", true);
            return runLine("CHAMFER" + named("body") + opt("distance", "DISTANCE") + (a["select"].isString() ? " EDGES " + a["select"].asString() : std::string()));
        }
        if (op == "edges") return runLine("EDGES" + named("body"));
        if (op == "massprops")
            return runLine("MASSPROPS" + named("body") + opt("density", "DENSITY") + (a["material"].isString() ? " MATERIAL " + a["material"].asString() : std::string()));
        if (op == "measure") {
            if (a["points"].isArray() && a["points"].size() == 2) {
                auto pt = [&](const Json& v) { return num(v[0].asNumber()) + "," + num(v[1].asNumber()) + "," + num(v[2].asNumber()); };
                return runLine("MEASURE " + pt(a["points"][0]) + " " + pt(a["points"][1]));
            }
            if (!a["other"].isString()) return textResult("measure needs 'points', or 'body' and 'other'", true);
            return runLine("MEASURE" + named("body") + named("other"));
        }
        if (op == "constrain" || op == "dim") {
            std::string line = op == "dim" ? "DIM" : "CONSTRAIN";
            const std::string kind = a["kind"].asString(op == "dim" ? "distance" : "");
            if (op == "constrain") {
                if (kind.empty()) return textResult("constrain needs 'kind'", true);
                line += " " + toUpper(kind);
            } else if (kind != "distance")
                line += " " + toUpper(kind);
            if (!a["refs"].isArray() || a["refs"].size() == 0) return textResult(op + " needs 'refs'", true);
            for (const Json& r : a["refs"].items()) {
                if (!r["curve"].isString()) return textResult("each ref needs 'curve'", true);
                line += " " + quote(r["curve"].asString());
                if (r["at"].isString()) line += " " + toUpper(r["at"].asString());
            }
            if (a["value"].isNumber()) line += " " + num(a["value"].asNumber());
            return runLine(line);
        }
        if (op == "constraints") return runLine("CONSTRAINTS");
        if (op == "drag") {
            if (!a["curve"].isString()) return textResult("drag needs 'curve'", true);
            auto xy = [&](const Json& v) { return num(v[0].asNumber()) + "," + num(v[1].asNumber()); };
            std::string line = "DRAG " + quote(a["curve"].asString());
            if (a["grip"].isString()) line += " " + toUpper(a["grip"].asString());
            if (a["by"].isArray() && a["by"].size() == 2) line += " BY " + xy(a["by"]);
            else if (a["to"].isArray() && a["to"].size() == 2) line += " TO " + xy(a["to"]);
            else return textResult("drag needs 'to' [x, y] (or 'by' [dx, dy] for a whole curve)", true);
            if (a["from"].isArray() && a["from"].size() == 2) line += " FROM " + xy(a["from"]);
            return runLine(line);
        }
        if (op == "constraint_edit" || op == "constraint_delete") {
            if (!a["constraint"].isString()) return textResult(op + " needs 'constraint'", true);
            if (op == "constraint_edit" && !a["value"].isNumber()) return textResult("constraint_edit needs 'value'", true);
            return runLine(std::string("CONSTRAINT ") + (op == "constraint_edit" ? "EDIT " : "DELETE ") + a["constraint"].asString() +
                           (op == "constraint_edit" ? " " + num(a["value"].asNumber()) : std::string()));
        }
        if (op == "history") return runLine("HISTORY");
        if (op == "regen") return runLine(std::string("REGEN") + (a["force"].asBool() ? " FORCE" : ""));
        if (op == "history_edit" || op == "history_insert" || op == "history_delete") {
            if (!a["step"].isNumber()) return textResult(op + " needs 'step'", true);
            const std::string what = op.substr(8);
            if (what != "delete" && !a["command"].isString()) return textResult(op + " needs 'command'", true);
            return runLine("HISTORY " + toUpper(what) + " " + num(a["step"].asNumber()) + (what != "delete" ? " " + a["command"].asString() : std::string()));
        }
        return textResult("operation is one of revolve, sweep, loft, pattern, mirror, edges, fillet, chamfer, massprops, measure, history, regen, "
                          "history_edit, history_insert, history_delete, constrain, dim, constraints, constraint_edit, constraint_delete, drag",
                          true);
    }
    if (name == "assembly") {
        const std::string op = a["operation"].asString();
        auto vec3 = [&](const Json& v) {
            std::string s;
            for (std::size_t i = 0; i < 3 && i < v.size(); ++i) s += (i ? "," : "") + num(v[i].asNumber());
            return s;
        };
        auto names = [&](const char* key) {
            std::string s;
            if (a[key].isArray())
                for (std::size_t i = 0; i < a[key].size(); ++i) s += " " + quote(a[key][i].asString());
            return s;
        };
        const std::string mate = a["mate"].isString() ? a["mate"].asString() : a["mate"].isNumber() ? num(a["mate"].asNumber()) : std::string();
        if (op == "mate") {
            if (!a["type"].isString() || !a["part"].isString() || !a["at"].isArray() || !a["moving"].isString() || !a["moving_at"].isArray())
                return textResult("mate needs type, part, at [x, y, z], moving and moving_at [x, y, z]", true);
            std::string line = "MATE " + a["type"].asString();
            if (a["value"].isNumber()) line += " " + num(a["value"].asNumber());
            line += " " + quote(a["part"].asString()) + " AT " + vec3(a["at"]) + " " + quote(a["moving"].asString()) + " AT " + vec3(a["moving_at"]);
            if (a["flip"].asBool()) line += " FLIP";
            return runLine(line);
        }
        if (op == "fix") {
            if (!a["part"].isString()) return textResult("fix needs part", true);
            return runLine("MATE FIX " + quote(a["part"].asString()));
        }
        if (op == "list") return runLine("MATE");
        if (op == "solve") return runLine("MATE SOLVE");
        if (op == "clear") return runLine("MATE CLEAR");
        if (op == "edit" || op == "flip" || op == "suppress" || op == "unsuppress" || op == "delete") {
            if (mate.empty()) return textResult(op + " needs mate (its name or number)", true);
            return runLine("MATE " + toUpper(op) + " " + mate + (op == "edit" ? " " + num(a["value"].asNumber()) : std::string()));
        }
        if (op == "tree") return runLine("ASSEMBLY");
        if (op == "group") {
            if (!a["name"].isString() || !a["parts"].isArray()) return textResult("group needs name and parts", true);
            return runLine("ASSEMBLY GROUP " + quote(a["name"].asString()) + names("parts"));
        }
        if (op == "ungroup") return runLine("ASSEMBLY UNGROUP" + names("parts"));
        if (op == "interference") return runLine("ASSEMBLY INTERFERENCE" + names("parts"));
        if (op == "bom") return runLine("ASSEMBLY BOM");
        return textResult("operation is one of mate, fix, list, solve, edit, flip, suppress, unsuppress, delete, clear, tree, group, ungroup, "
                          "interference, bom",
                          true);
    }
    if (name == "drawing") {
        const std::string op = a["operation"].asString();
        auto vec = [&](const Json& v, std::size_t n) {
            std::string s;
            for (std::size_t i = 0; i < n && i < v.size(); ++i) s += (i ? "," : "") + num(v[i].asNumber());
            return s;
        };
        auto text = [&](const char* key) { return a[key].isString() ? " " + quote(a[key].asString()) : std::string(); };
        const std::string view = text("view");
        if (op == "list") return runLine("SHEET");
        if (op == "delete") return runLine("SHEET DELETE");
        if (op == "autodim") return runLine("SHEET AUTODIM" + view);
        if (op == "new") {
            std::string line = "SHEET NEW";
            if (a["bodies"].isArray())
                for (std::size_t i = 0; i < a["bodies"].size(); ++i) line += " " + quote(a["bodies"][i].asString());
            if (a["size"].isString()) line += " " + a["size"].asString();
            if (a["portrait"].asBool()) line += " PORTRAIT";
            if (a["third_angle"].asBool()) line += " THIRD";
            if (a["scale"].isString()) line += " SCALE " + a["scale"].asString();
            return runLine(line);
        }
        if (op == "view_add") {
            if (!a["view"].isString()) return textResult("view_add needs view (front, top, right, iso...)", true);
            std::string line = "SHEET VIEW ADD " + a["view"].asString();
            if (a["at"].isArray()) line += " AT " + vec(a["at"], 2);
            if (a["scale"].isString()) line += " SCALE " + a["scale"].asString();
            if (a["hidden"].isBool()) line += a["hidden"].asBool() ? " HIDDEN" : " NOHIDDEN";
            return runLine(line);
        }
        if (op == "view_move") {
            if (!a["view"].isString() || !a["at"].isArray()) return textResult("view_move needs view and at [x, y]", true);
            return runLine("SHEET VIEW MOVE" + view + " " + vec(a["at"], 2));
        }
        if (op == "view_delete") return runLine("SHEET VIEW DELETE" + view);
        if (op == "view_scale") return runLine("SHEET VIEW SCALE" + view + " " + a["scale"].asString());
        if (op == "view_hidden") return runLine("SHEET VIEW HIDDEN" + view + (a["hidden"].asBool() ? " ON" : " OFF"));
        if (op == "section") {
            std::string line = "SHEET SECTION" + view + (a["horizontal"].asBool() ? " HORIZONTAL" : "");
            if (a["at"].isNumber()) line += " AT " + num(a["at"].asNumber());
            if (a["flip"].asBool()) line += " FLIP";
            return runLine(line);
        }
        if (op == "dim") {
            if (!a["kind"].isString() || !a["view"].isString() || !a["from"].isArray() || !a["to"].isArray())
                return textResult("dim needs kind, view, from [x, y, z] and to [x, y, z]", true);
            std::string line = "SHEET DIM " + a["kind"].asString() + view + " " + vec(a["from"], 3) + " " + vec(a["to"], 3);
            if (a["offset"].isNumber()) line += " OFFSET " + num(a["offset"].asNumber());
            if (a["tolerance"].isString()) line += " TOL " + a["tolerance"].asString();
            if (a["text"].isString()) line += " TEXT " + quote(a["text"].asString());
            return runLine(line);
        }
        if (op == "tolerance") {
            if (!a["index"].isNumber() || !a["tolerance"].isString()) return textResult("tolerance needs index (the dimension) and tolerance", true);
            return runLine("SHEET TOL " + num(a["index"].asNumber()) + " " + a["tolerance"].asString());
        }
        if (op == "general_tolerance") {
            if (!a["tolerance"].isString()) return textResult("general_tolerance needs tolerance: \"ISO 2768-m\" or \"NONE\"", true);
            return runLine("SHEET TOLERANCE " + a["tolerance"].asString());
        }
        auto placement = [&]() -> std::string {
            std::string s;
            if (a["index"].isNumber() && op == "gdt") return " DIM " + num(a["index"].asNumber());
            if (a["point"].isArray()) s = " ON" + view + " " + vec(a["point"], 3);
            if (a["sheet_offset"].isArray()) s += " OFFSET " + vec(a["sheet_offset"], 2);
            return s;
        };
        if (op == "gdt") {
            if (!a["symbol"].isString() || !a["value"].isNumber()) return textResult("gdt needs symbol, value, and view + point (or index of a dimension)", true);
            std::string line = "SHEET GDT " + a["symbol"].asString() + " " + num(a["value"].asNumber());
            if (a["diameter"].asBool()) line += " DIA";
            if (a["material"].isString()) line += " " + toUpper(a["material"].asString());
            if (a["datums"].isArray() && a["datums"].size() > 0) {
                line += " DATUMS";
                for (const Json& x : a["datums"].items()) line += " " + x.asString();
            }
            return runLine(line + placement());
        }
        if (op == "gdt_delete") return runLine("SHEET GDT DELETE " + num(a["index"].asNumber()));
        if (op == "datum") {
            if (!a["letter"].isString()) return textResult("datum needs letter, view and point", true);
            return runLine("SHEET DATUM " + a["letter"].asString() + placement());
        }
        if (op == "datum_delete") return runLine("SHEET DATUM DELETE " + a["letter"].asString());
        if (op == "finish") {
            if (!a["value"].isNumber()) return textResult("finish needs value (Ra, micrometres), view and point", true);
            std::string line = "SHEET FINISH RA " + num(a["value"].asNumber());
            if (a["machined"].isBool()) line += a["machined"].asBool() ? " MACHINED" : " UNMACHINED";
            return runLine(line + placement());
        }
        if (op == "dim_delete") return runLine("SHEET DIM DELETE " + num(a["index"].asNumber()));
        if (op == "title") {
            if (!a["field"].isString()) return textResult("title needs field and text", true);
            return runLine("SHEET TITLE " + a["field"].asString() + text("text"));
        }
        if (op == "note") {
            std::string line = "SHEET NOTE" + text("text");
            if (a["at"].isArray()) line += " AT " + vec(a["at"], 2);
            return runLine(line);
        }
        if (op == "scale") return runLine("SHEET SCALE " + a["scale"].asString());
        if (op == "size") return runLine("SHEET SIZE " + a["size"].asString() + (a["portrait"].asBool() ? " PORTRAIT" : ""));
        if (op == "projection") return runLine(std::string("SHEET PROJECTION ") + (a["third_angle"].asBool() ? "THIRD" : "FIRST"));
        if (op == "export") {
            if (!a["path"].isString()) return textResult("export needs path (.pdf, .svg or .dxf)", true);
            return runLine("SHEET EXPORT " + quote(a["path"].asString()));
        }
        return textResult("operation is one of new, list, view_add, view_move, view_delete, view_scale, view_hidden, section, dim, dim_delete, "
                          "autodim, tolerance, gdt, gdt_delete, datum, datum_delete, finish, general_tolerance, title, note, scale, size, "
                          "projection, export, delete",
                          true);
    }
    if (name == "lathe") {
        const std::string op = a["operation"].asString();
        auto opt = [&](const char* key, const char* word) {
            return a[key].isNumber() ? std::string(" ") + word + " " + num(a[key].asNumber()) : std::string();
        };
        auto text = [&](const char* key, const char* word) {
            return a[key].isString() ? std::string(" ") + word + " " + quote(a[key].asString()) : std::string();
        };
        if (op == "setup") {
            std::string line = "LATHE SETUP" + (a["body"].isString() ? " " + quote(a["body"].asString()) : std::string());
            if (a["axis"].isString()) line += " AXIS " + a["axis"].asString();
            if (a["flip"].asBool()) line += " FLIP";
            line += opt("bar", "BAR") + opt("front", "FRONT") + opt("max_rpm", "MAXRPM");
            if (a["material"].isString()) line += " MATERIAL " + a["material"].asString();
            if (a["post"].isString()) line += " POST " + a["post"].asString();
            return runLine(line);
        }
        if (op == "tool") {
            if (!a["number"].isNumber() || !a["type"].isString()) return textResult("tool needs number and type", true);
            std::string line = "LATHE TOOL " + num(a["number"].asNumber()) + " " + a["type"].asString();
            if (a["size"].isNumber()) line += " " + num(a["size"].asNumber());
            line += opt("nose", "NOSE") + opt("reach", "REACH") + text("name", "NAME");
            if (a["insert"].isString() || a["insert"].isNumber()) line += " INSERT " + (a["insert"].isNumber() ? num(a["insert"].asNumber()) : a["insert"].asString());
            return runLine(line);
        }
        if (op == "tools" || op == "profile" || op == "list" || op == "generate" || op == "clear") return runLine("LATHE " + toUpper(op));
        if (op == "simulate") return runLine(std::string("LATHE SIMULATE") + (a["show"].asBool() ? " SHOW" : ""));
        if (op == "post") {
            if (!a["path"].isString()) return textResult("post needs 'path'", true);
            return runLine("LATHE POST " + quote(a["path"].asString()) + (a["post"].isString() ? " " + a["post"].asString() : std::string()));
        }
        if (op == "delete") {
            if (!a["index"].isNumber()) return textResult("delete needs 'index'", true);
            return runLine("LATHE DELETE " + num(a["index"].asNumber()));
        }
        static const char* const kOps[] = {"face", "rough", "finish", "bore", "groove", "drill", "thread", "partoff"};
        if (std::find(std::begin(kOps), std::end(kOps), op) == std::end(kOps))
            return textResult("operation is one of setup, tool, tools, profile, face, rough, finish, bore, groove, drill, thread, partoff, list, generate, "
                              "simulate, post, delete, clear",
                              true);
        std::string line = "LATHE " + toUpper(op) + opt("tool", "TOOL") + opt("stepdown", "STEPDOWN") + opt("leave", "LEAVE") + opt("leave_z", "LEAVEZ") +
                           opt("from", "FROM") + opt("to", "TO") + opt("depth", "DEPTH") + opt("peck", "PECK") + opt("pitch", "PITCH") +
                           opt("diameter", "DIAMETER") + opt("speed", "SPEED") + opt("feed", "FEED") + opt("rpm", "RPM") + text("name", "NAME");
        if (a["inside"].asBool()) line += " INSIDE";
        return runLine(line);
    }
    if (name == "cam") {
        const std::string op = a["operation"].asString();
        auto opt = [&](const char* key, const char* word) {
            return a[key].isNumber() ? std::string(" ") + word + " " + num(a[key].asNumber()) : std::string();
        };
        auto text = [&](const char* key, const char* word) {
            return a[key].isString() ? std::string(" ") + word + " " + quote(a[key].asString()) : std::string();
        };
        if (op == "library") {
            const std::string what = a["library"].asString("list");
            std::string line = "CAM LIBRARY " + toUpper(what);
            if (what == "import" || what == "export" || what == "load" || what == "save") {
                if (!a["path"].isString()) return textResult("library " + what + " needs path", true);
                line += " " + quote(a["path"].asString());
                if (a["file_format"].isString()) line += " " + toUpper(a["file_format"].asString());
            } else if (a["entry"].isString() || a["entry"].isNumber())
                line += " " + (a["entry"].isNumber() ? num(a["entry"].asNumber()) : a["entry"].asString());
            if (what == "use" && a["as"].isNumber()) line += " AS " + num(a["as"].asNumber());
            return runLine(line);
        }
        if (op == "setup") {
            std::string line = "CAM SETUP" + (a["body"].isString() ? " " + quote(a["body"].asString()) : std::string());
            if (a["stock_size"].isArray() && a["stock_size"].size() == 3)
                line += " STOCK " + num(a["stock_size"][0].asNumber()) + "," + num(a["stock_size"][1].asNumber()) + "," + num(a["stock_size"][2].asNumber());
            line += opt("stock_margin", "STOCK");
            if (a["material"].isString()) line += " MATERIAL " + a["material"].asString();
            if (a["origin"].isString()) line += " ORIGIN " + a["origin"].asString();
            if (a["post"].isString()) line += " POST " + a["post"].asString();
            return runLine(line);
        }
        if (op == "tool") {
            if (!a["number"].isNumber() || !a["type"].isString() || !a["diameter"].isNumber())
                return textResult("tool needs number, type and diameter", true);
            return runLine("CAM TOOL " + num(a["number"].asNumber()) + " " + a["type"].asString() + " " + num(a["diameter"].asNumber()) +
                           opt("corner", "CORNER") + opt("flutes", "FLUTES") + opt("length", "LENGTH") + opt("stickout", "STICKOUT") + opt("holder", "HOLDER") +
                           text("name", "NAME"));
        }
        if (op == "holes" || op == "list" || op == "generate" || op == "clear") return runLine("CAM " + toUpper(op));
        if (op == "simulate") return runLine(std::string("CAM SIMULATE") + (a["show"].asBool() ? " SHOW" : ""));
        if (op == "machine") return runLine("CAM MACHINE" + (a["preset"].isString() ? " PRESET " + a["preset"].asString() : std::string()));
        if (op == "post") {
            if (!a["path"].isString()) return textResult("post needs 'path'", true);
            return runLine("CAM POST " + quote(a["path"].asString()) + (a["post"].isString() ? " " + a["post"].asString() : std::string()));
        }
        if (op == "delete") {
            if (!a["index"].isNumber()) return textResult("delete needs 'index'", true);
            return runLine("CAM DELETE " + num(a["index"].asNumber()));
        }
        static const char* const kOps[] = {"face", "rough", "pocket", "contour", "parallel", "waterline", "drill", "bore", "rotary", "5axis"};
        if (std::find(std::begin(kOps), std::end(kOps), op) == std::end(kOps))
            return textResult("operation is one of setup, tool, face, rough, pocket, contour, parallel, waterline, drill, bore, rotary, 5axis, machine, holes, "
                              "list, generate, simulate, post, delete, clear",
                              true);
        std::string line = "CAM " + toUpper(op) + opt("tool", "TOOL") + opt("stepdown", "STEPDOWN") + opt("stepover", "STEPOVER") +
                           opt("leave", "LEAVE") + opt("top", "TOP") + opt("bottom", "BOTTOM") + opt("depth", "DEPTH") + opt("tabs", "TABS") +
                           opt("scallop", "SCALLOP") + opt("peck", "PECK") + opt("diameter", "DIAMETER") + opt("max_tilt", "MAXTILT") +
                           opt("lead_angle", "LEADANGLE") + opt("side_tilt", "SIDETILT") + text("name", "NAME");
        if (a["stepover_percent"].isNumber()) line += " STEPOVER " + num(a["stepover_percent"].asNumber()) + "%";
        if (a["side"].isString()) line += " " + toUpper(a["side"].asString());
        if (a["raster"].asBool()) line += " RASTER";
        if (a["adaptive"].asBool()) line += " ADAPTIVE";
        if (a["rest"].asBool()) line += " REST";
        if (a["along"].asBool()) line += " ALONG";
        if (a["index_a"].isNumber() || a["index_c"].isNumber())
            line += " INDEX " + num(a["index_a"].asNumber(0)) + " " + num(a["index_c"].asNumber(0));
        if (a["cycle"].isString()) line += " CYCLE " + a["cycle"].asString();
        if (a["slope"].isArray() && a["slope"].size() == 2) line += " SLOPE " + num(a["slope"][0].asNumber()) + " " + num(a["slope"][1].asNumber());
        if (a["points"].isArray() && a["points"].size() > 0) {
            line += " AT";
            for (const Json& q : a["points"].items()) line += " " + num(q[0].asNumber()) + "," + num(q[1].asNumber());
        }
        return runLine(line);
    }
    if (name == "armor") {
        const std::string op = a["operation"].asString();
        auto vec = [&](const Json& v) { return num(v[0].asNumber()) + "," + num(v[1].asNumber()) + "," + num(v[2].asNumber()); };
        const std::string body = a["body"].isString() ? quote(a["body"].asString()) : "";
        auto opt = [&](const char* key, const char* word) {
            return a[key].isNumber() ? std::string(" ") + word + " " + num(a[key].asNumber()) : std::string();
        };
        std::string line;
        if (op == "shell") {
            line = "SHELL " + body + opt("thickness", "THICKNESS") + (a["outward"].asBool() ? " OUTWARD" : "");
            if (a["open_point"].isArray())
                line += " OPEN " + vec(a["open_point"]) + (a["open_normal"].isArray() ? " NORMAL " + vec(a["open_normal"]) : "");
        } else if (op == "armor") {
            line = "ARMOR " + body + (a["part"].isString() ? " PART " + quote(a["part"].asString()) : "") +
                   (a["part"].isString() && a["merge"].asBool() ? " MERGE" : "") + opt("clearance", "CLEARANCE") +
                   opt("thickness", "THICKNESS");
        } else if (op == "joint") {
            if (!a["at"].isArray()) return textResult("joint needs 'at'", true);
            line = "JOINT " + body + (a["parent"].isString() ? " TO " + quote(a["parent"].asString()) : "") + " AT " + vec(a["at"]) +
                   (a["axis"].isArray() ? " AXIS " + vec(a["axis"]) : "") + (a["ball"].asBool() ? " BALL" : "");
            if (a["range"].isArray()) line += " RANGE " + num(a["range"][0].asNumber()) + " " + num(a["range"][1].asNumber());
            line += opt("period", "PERIOD");
        } else if (op == "lighten") {
            line = "LIGHTEN " + body + opt("skin", "SKIN") + opt("cell", "CELL") + opt("wall", "WALL") + (a["hollow"].asBool() ? " HOLLOW" : "");
        } else if (op == "interference") {
            line = "INTERFERENCE" + opt("clearance", "CLEARANCE") + opt("step", "STEP") + opt("duration", "DURATION");
            if (a["bodies"].isArray() && a["bodies"].size() > 0) {
                line += " BODIES";
                for (std::size_t i = 0; i < a["bodies"].size(); ++i) line += " " + quote(a["bodies"][i].asString());
            }
        } else {
            return textResult("operation must be shell, armor, joint, lighten or interference", true);
        }
        return runLine(line);
    }
    if (name == "sculpt") {
        const std::string op = a["operation"].asString();
        auto vec = [&](const Json& v) { return num(v[0].asNumber()) + "," + num(v[1].asNumber()) + "," + num(v[2].asNumber()); };
        if (op == "undo") return runLine("UNDO");
        if (op == "redo") return runLine("REDO");
        std::string body = a["body"].isString() ? quote(a["body"].asString()) : "";
        std::string pts;
        if (a["points"].isArray())
            for (std::size_t i = 0; i < a["points"].size(); ++i) pts += " " + vec(a["points"][i]);
        auto opt = [&](const char* key, const char* word) {
            return a[key].isNumber() ? std::string(" ") + word + " " + num(a[key].asNumber()) : std::string();
        };
        std::string line;
        if (op == "boolean") {
            if (!a["body"].isString() || !a["other"].isString()) return textResult("boolean needs 'body' and 'other'", true);
            line = "BOOLEAN " + body + " " + a["boolean_op"].asString("subtract") + " " + quote(a["other"].asString()) +
                   (a["keep"].asBool() ? " KEEP" : "");
        } else if (op == "subdivide") {
            line = "SUBDIVIDE " + body + " LEVELS " + num(a["levels"].asNumber(1));
        } else if (op == "symmetrize") {
            line = "SYMMETRIZE " + body + " " + a["axis"].asString("x");
        } else if (op == "decimate") {
            line = "DECIMATE " + body + " TO " + num(a["target"].asNumber(10000));
        } else if (op == "mask_clear" || op == "mask_invert" || op == "mask_smooth") {
            line = "MASK " + body + " " + op.substr(5);
        } else if (op == "remesh") {
            line = "REMESH " + body;
            if (a["others"].isArray())
                for (std::size_t i = 0; i < a["others"].size(); ++i) line += " " + quote(a["others"][i].asString());
            line += opt("voxel", "VOXEL") + opt("blend", "BLEND") + opt("smooth", "SMOOTH");
        } else if (op == "refine") {
            line = "SCULPT " + body + " REFINE " + num(a["detail"].asNumber(2));
        } else if (op == "hole") {
            line = "HOLE " + body + " AT" + pts + opt("diameter", "DIAMETER") + opt("depth", "DEPTH");
        } else if (op == "pocket") {
            line = "POCKET " + body + " AT" + pts + " " + a["shape"].asString("box") + opt("width", "WIDTH") +
                   opt("length", "LENGTH") + opt("depth", "DEPTH") + opt("angle", "ANGLE");
        } else if (op == "channel") {
            line = "CHANNEL " + body + " PATH" + pts + opt("width", "WIDTH") + opt("depth", "DEPTH");
        } else if (op == "trim") {
            line = "TRIM " + body + " PLANE" + pts + (a["normal"].isArray() ? " NORMAL " + vec(a["normal"]) : "") +
                   (a["smaller"].asBool() ? " SMALLER" : "");
        } else if (op == "boss") {
            line = "BOSS " + body + " AT" + pts + " " + a["shape"].asString("cylinder") + opt("diameter", "DIAMETER") +
                   opt("height", "HEIGHT");
        } else {
            line = "SCULPT " + body + " " + op + " AT" + pts + opt("radius", "RADIUS") + opt("strength", "STRENGTH") +
                   opt("detail", "DETAIL") + (a["invert"].asBool() ? " INVERT" : "") +
                   (a["symmetry_axes"].isString() ? " SYMMETRY " + a["symmetry_axes"].asString() : a["symmetry"].asBool() ? " SYMMETRY" : "") +
                   (a["radial"].isNumber() ? " RADIAL " + num(a["radial"].asNumber()) + " " + a["radial_axis"].asString("z") : "") +
                   (a["falloff"].isString() ? " FALLOFF " + a["falloff"].asString() : "") +
                   (a["move"].isArray() ? " MOVE " + vec(a["move"]) : "");
        }
        if (a["direction"].isArray() && (op == "hole" || op == "pocket" || op == "boss" || op == "channel"))
            line += " DIR " + vec(a["direction"]);
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

#include "core/Mcp.hpp"

#include "core/Command.hpp"

#include <algorithm>
#include <cstdio>

namespace sicdm {

namespace {

constexpr const char* kServerVersion = "0.1.0";
constexpr const char* kSupportedVersions[] = {"2025-06-18", "2025-03-26", "2024-11-05"};

constexpr const char* kInstructions =
    "SI CDM is a CAD/CAM workbench. Units are millimetres; Z is up. The model has 3D bodies "
    "(parametric solids or imported meshes), 2D sketch curves on the Top (XY), Front (XZ) or Right (YZ) "
    "plane, and reference images on those planes. Call get_scene first to see what exists. "
    "Sketch coordinates are 2D in the plane: Top u=X v=Y, Front u=X v=Z, Right u=Y v=Z. "
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
        p.set("name", prop("string", "Body, curve or image name (see get_scene)"));
        p.set("position", numberArray(3, "Body: new [x,y,z] position; image: [u,v,ignored] centre"));
        p.set("move_by", numberArray(3, "Body: [dx,dy,dz]; image: [du,dv,ignored]"));
        p.set("rename", prop("string", "New name"));
        p.set("visible", prop("boolean", "Show or hide"));
        p.set("color", numberArray(3, "Body colour [r,g,b] in 0..1"));
        p.set("width", prop("number", "Image width (mm)"));
        p.set("opacity", prop("number", "Image opacity 0..1"));
        tools.push(tool("modify", "Move, rename, recolour, show/hide a body, curve or image.", p, {"name"}));
    }
    {
        Json p = Json::object();
        p.set("name", prop("string", "Name of the body, curve or image to delete"));
        tools.push(tool("delete", "Delete a body, sketch curve or image.", p, {"name"}));
    }
    {
        Json p = Json::object();
        p.set("window", enumProp({"2d", "3d"}, "2D Drawing window or 3D Model window"));
        p.set("plane", enumProp({"top", "front", "right"}, "Sketch / drawing plane"));
        p.set("view", enumProp({"top", "bottom", "front", "back", "left", "right", "iso"}, "Standard view (3D)"));
        p.set("projection", enumProp({"perspective", "orthographic"}, "3D projection"));
        p.set("display", enumProp({"shaded", "edges", "wireframe", "hidden"}, "3D display mode"));
        p.set("roll", prop("number", "Roll the 3D view by degrees (0 levels it)"));
        p.set("fit", prop("boolean", "Zoom to fit everything"));
        tools.push(tool("set_view", "Change the window, plane, camera view or display mode of the app.", p, {}));
    }
    {
        Json p = Json::object();
        p.set("path", prop("string", "File path (.sicdm). Optional for save when the project already has a path"));
        tools.push(tool("save_project", "Save the project.", p, {}));
        Json o = Json::object();
        o.set("path", prop("string", "Project file to open (.sicdm)"));
        tools.push(tool("open_project", "Open a project, replacing the current document.", o, {"path"}));
        tools.push(tool("new_project", "Start an empty project (unsaved changes are discarded).", Json::object(), {}));
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
    if (name == "set_view") {
        std::string log;
        auto step = [&](const std::string& line) {
            const Json r = runLine(line);
            log += (log.empty() ? "" : "\n") + r["content"][0]["text"].asString();
            return !r["isError"].asBool();
        };
        bool ok = true;
        if (a["plane"].isString()) ok = step("PLANE " + a["plane"].asString()) && ok;
        if (a["window"].isString()) ok = step(a["window"].asString() == "2d" ? "2D" : "3D") && ok;
        if (a["view"].isString()) ok = step("VIEW " + a["view"].asString()) && ok;
        if (a["projection"].isString()) ok = step(a["projection"].asString() == "orthographic" ? "ORTHO" : "PERSP") && ok;
        if (a["display"].isString()) ok = step("DISPLAY " + a["display"].asString()) && ok;
        if (a["roll"].isNumber()) ok = step("ROLL " + num(a["roll"].asNumber())) && ok;
        if (a["fit"].asBool()) ok = step("FIT") && ok;
        if (log.empty()) return textResult("Nothing to change", true);
        return textResult(log, !ok);
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
        Json result = callTool(name, args);
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

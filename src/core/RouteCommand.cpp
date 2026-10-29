// PATH3D: 3D sketch curves (polylines with bends, splines). ROUTE: pipes, tubes, bars,
// cables and hoses along them, and their bend tables.

#include "core/RouteCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/ExactMesh.hpp"
#include "core/Route.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <optional>

namespace sicdm {

namespace {

constexpr double kDeg = 180.0 / 3.14159265358979323846;

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string num(double v, int digits = 4) {
    char b[32];
    std::snprintf(b, sizeof b, "%.*g", digits, std::fabs(v) < 5e-10 ? 0.0 : v);
    return b;
}

std::string pointText(Vec3 p) { return lengthsText({p.x, p.y, p.z}, ", ", "(", ")"); }

bool unquotedIs(const std::vector<std::string>& a, const std::vector<bool>& q, std::size_t i, const char* w) {
    return i < a.size() && !(i < q.size() && q[i]) && lowerOf(a[i]) == w;
}

int curveIndex(const Document& d, const std::string& n) {
    const auto& c = d.scene.sketch().curves();
    for (std::size_t i = 0; i < c.size(); ++i)
        if (c[i].name == n) return static_cast<int>(i);
    for (std::size_t i = 0; i < c.size(); ++i)
        if (lowerOf(c[i].name) == lowerOf(n)) return static_cast<int>(i);
    return -1;
}

int bodyIndex(const Document& d, const std::string& n) {
    const auto& b = d.scene.bodies();
    for (std::size_t i = 0; i < b.size(); ++i)
        if (b[i].name == n) return static_cast<int>(i);
    for (std::size_t i = 0; i < b.size(); ++i)
        if (lowerOf(b[i].name) == lowerOf(n)) return static_cast<int>(i);
    return -1;
}

// The name at a[i]: a quoted word, or the longest run of words that `found` accepts (up
// to a word `stop` says ends it). Moves i past it; "" when none.
template <class Found, class Stop>
std::string leadingName(const std::vector<std::string>& a, const std::vector<bool>& q, std::size_t& i, Found found, Stop stop) {
    if (i >= a.size()) return {};
    if (i < q.size() && q[i]) return a[i++];
    std::string name, best;
    std::size_t end = i;
    for (std::size_t j = i; j < a.size() && !(j < q.size() && q[j]) && !stop(lowerOf(a[j])); ++j) {
        name += (name.empty() ? "" : " ") + a[j];
        if (found(name)) best = name, end = j + 1;
    }
    if (!best.empty()) i = end;
    return best;
}

// A curve's points in space (a closed polyline's first not repeated).
std::vector<Vec3> curvePoints(const SketchCurve& c, bool& closed) {
    if (c.kind == CurveKind::Rectangle && c.pts.size() >= 2) {
        closed = true;
        const PlaneFrame f = curveFrame(c);
        const Vec2 a = c.pts[0], b = c.pts[1];
        return {f.toWorld(a), f.toWorld({b.x, a.y}), f.toWorld(b), f.toWorld({a.x, b.y})};
    }
    return spacePoints(c, &closed);
}

// Puts points in space on a 3D curve (its plane's coordinates and heights).
void setSpacePoints(SketchCurve& c, const std::vector<Vec3>& p, bool closed) {
    const PlaneFrame f = curveFrame(c);
    c.pts.clear(), c.rise.clear();
    for (const Vec3& q : p) {
        c.pts.push_back(f.toPlane(q));
        c.rise.push_back(dot(q - f.origin, f.normal));
    }
    if (c.kind == CurveKind::Polyline && closed && !p.empty()) c.pts.push_back(c.pts.front()), c.rise.push_back(c.rise.front());
    c.closed = c.kind == CurveKind::Spline && closed;
}

// What a 3D curve is: its points numbered, segments, corners and length.
void describePath(const SketchCurve& c, CommandOutput& out) {
    bool closed = false;
    const std::vector<Vec3> p = spacePoints(c, &closed);
    out.say(c.name + ": a 3D " + (c.kind == CurveKind::Spline ? "spline" : "polyline") + " through " + std::to_string(p.size()) + " points" +
            (closed ? ", closed" : "") + (c.kind == CurveKind::Polyline && c.radius > 0 ? ", corners bent at R " + lengthText(c.radius) : "") + "  " + c.measure());
    for (std::size_t k = 0; k < p.size(); ++k) {
        std::string line = "  " + std::to_string(k + 1) + ". " + pointText(p[k]);
        const bool corner = closed || (k > 0 && k + 1 < p.size());
        if (k + 1 < p.size() || closed) line += "  -> " + lengthText(length(p[(k + 1) % p.size()] - p[k]));
        if (corner && c.kind == CurveKind::Polyline) {
            const Vec3 a = normalize(p[k] - p[(k + p.size() - 1) % p.size()]), b = normalize(p[(k + 1) % p.size()] - p[k]);
            const double turnDeg = std::acos(std::clamp(static_cast<double>(dot(a, b)), -1.0, 1.0)) * kDeg;
            if (turnDeg > 0.01) line += "  turns " + num(turnDeg) + " deg";
        }
        out.say(line);
    }
    if (c.kind == CurveKind::Polyline && c.radius > 0) {
        std::vector<PathBend> bends;
        std::string why;
        if (!pathBends(p, closed, c.radius, bends, &why)) out.say("  The bends don't fit: " + why);
    }
}

std::string routeKindName(const std::string& kind) {
    if (kind.empty()) return "Route";
    std::string k = kind;
    k[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(k[0])));
    return k;
}

std::string newRouteName(const Document& d, const std::string& kind) {
    const std::string base = routeKindName(kind);
    for (int k = 1;; ++k)
        if (bodyIndex(d, base + " " + std::to_string(k)) < 0) return base + " " + std::to_string(k);
}

// The description and material the parts list shows for a route.
void labelRoute(Body& b) {
    const auto spec = routeSpec(b.route.spec);
    const RouteReport rep = routeReport(b.route);
    b.properties["DESCRIPTION"] = (spec ? spec->designation : b.route.spec) + ", " + lengthText(rep.length) + (rep.bends.empty() ? "" : ", " + std::to_string(rep.bends.size()) + (rep.bends.size() == 1 ? " bend" : " bends"));
    b.properties["MATERIAL"] = b.route.material;
}

// (Re)makes a route body's solid from its record, at the path's place.
bool remakeRoute(Body& b, std::string* why) {
    MeshData mesh;
    std::vector<BSurface> surfaces;
    if (!buildRoute(b.route, mesh, surfaces, why)) return false;
    b.mesh = std::move(mesh);
    b.kind = "mesh";
    b.params.clear();
    b.offset = {}, b.rotation = {}, b.scale = 1.0f;
    b.brep.reset();
    keepExactGeometry(b, surfaces);
    b.route.key = meshKey(b.mesh);
    labelRoute(b);
    return true;
}

// The path a route takes from a sketch curve: its points, spline or not, closed or not.
bool pathFromCurve(const SketchCurve& c, Body::Route& r, std::string* why) {
    const bool ok = c.kind == CurveKind::Line || c.kind == CurveKind::Polyline || c.kind == CurveKind::Spline || c.kind == CurveKind::Rectangle;
    if (!ok) {
        if (why) *why = c.name + " can't be a route's path: routes follow lines, polylines, rectangles and splines, flat or 3D (PATH3D)";
        return false;
    }
    bool closed = false;
    r.points = curvePoints(c, closed);
    r.closed = closed;
    r.spline = c.kind == CurveKind::Spline;
    r.path = c.name;
    return true;
}

bool samePoints(const std::vector<Vec3>& a, const std::vector<Vec3>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (length(a[i] - b[i]) > 1e-4f) return false;
    return true;
}

// Whether a route still lies along its path: "" when it does, else what changed.
std::string pathStatus(const Document& d, const Body& b) {
    const int ci = curveIndex(d, b.route.path);
    if (ci < 0) return "its path " + b.route.path + " is gone";
    Body::Route now = b.route;
    if (!pathFromCurve(d.scene.sketch().curves()[static_cast<std::size_t>(ci)], now, nullptr)) return "its path is no longer one";
    if (!samePoints(now.points, b.route.points) || now.closed != b.route.closed || now.spline != b.route.spline) return "its path changed (ROUTE UPDATE)";
    return {};
}

} // namespace

CommandOutput runPath3dCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    Sketch& sketch = doc.scene.sketch();
    const std::string sub = a.empty() || (!quoted.empty() && quoted[0]) ? std::string() : lowerOf(a[0]);
    auto isNumber = [](const std::string& w) { return parseLength(w).has_value(); };

    if (sub == "point" || sub == "insert" || sub == "delete" || sub == "bend" || sub == "info") {
        // PATH3D POINT|INSERT curve n x,y,z | DELETE curve n | BEND curve r | INFO [curve]
        std::size_t i = 1;
        const std::string name = leadingName(a, quoted, i, [&](const std::string& n) { return curveIndex(doc, n) >= 0; }, [&](const std::string& w) { return isNumber(w); });
        int ci = name.empty() ? doc.selectedCurve : curveIndex(doc, name);
        if (name.empty() && i < a.size() && !isNumber(a[i])) return out.fail("No sketch curve named " + a[i]);
        if (ci < 0 || ci >= static_cast<int>(sketch.curves().size())) return out.fail("Which 3D curve? Name it, or select it");
        SketchCurve& c = sketch.curves()[static_cast<std::size_t>(ci)];
        if (!c.spatial()) return out.fail(c.name + " is a flat curve: PATH3D edits 3D curves (PATH3D x,y,z x,y,z ... makes one)");
        if (sub == "info") {
            describePath(c, out);
            return out;
        }
        bool closed = false;
        std::vector<Vec3> p = spacePoints(c, &closed);
        const float radiusBefore = c.radius;
        auto index = [&](std::size_t at, std::size_t count) -> std::optional<std::size_t> {
            if (at >= a.size()) return std::nullopt;
            const auto n = parseNumber(a[at]);
            if (!n || *n < 1 || *n > static_cast<double>(count) || std::floor(*n) != *n) return std::nullopt;
            return static_cast<std::size_t>(*n) - 1;
        };
        std::string said;
        if (sub == "bend") {
            if (c.kind != CurveKind::Polyline) return out.fail("A spline is smooth: BEND rounds a 3D polyline's corners");
            const auto r = i < a.size() ? parseLength(a[i]) : std::nullopt;
            if (!r || *r < 0) return out.fail("PATH3D BEND curve r (0: sharp corners)");
            c.radius = static_cast<float>(*r);
            said = *r > 0 ? "Bends of " + c.name + " at R " + lengthText(*r) : c.name + "'s corners sharp";
        } else if (sub == "delete") {
            const auto k = index(i, p.size());
            if (!k) return out.fail("PATH3D DELETE curve n (n from 1 to " + std::to_string(p.size()) + ")");
            const std::size_t least = c.kind == CurveKind::Spline && closed ? 3 : closed ? 3 : 2;
            if (p.size() <= least) return out.fail(c.name + " can't lose a point: it would have fewer than " + std::to_string(least));
            p.erase(p.begin() + static_cast<std::ptrdiff_t>(*k));
            said = "Took point " + std::to_string(*k + 1) + " out of " + c.name;
        } else {
            const bool insert = sub == "insert";
            const auto k = index(i, p.size() + (insert ? 1 : 0));
            if (!k || i + 1 >= a.size()) return out.fail(std::string("PATH3D ") + (insert ? "INSERT curve n x,y,z (before point n; n = count + 1 adds at the end)" : "POINT curve n x,y,z (or @dx,dy,dz)"));
            const bool rel = !a[i + 1].empty() && a[i + 1][0] == '@';
            const auto v = parseVec3(a[i + 1]);
            if (!v) return out.fail("Not a point: " + a[i + 1] + "  (x,y,z or @dx,dy,dz)");
            if (insert) {
                const Vec3 from = *k > 0 ? p[*k - 1] : p[0];
                p.insert(p.begin() + static_cast<std::ptrdiff_t>(*k), rel ? from + *v : *v);
                said = "Put a point " + std::to_string(*k + 1) + " into " + c.name + " at " + pointText(p[*k]);
            } else {
                p[*k] = rel ? p[*k] + *v : *v;
                said = "Moved point " + std::to_string(*k + 1) + " of " + c.name + " to " + pointText(p[*k]);
            }
        }
        // Still a path: no two points on top of each other, the bends fitting.
        std::vector<PathBend> bends;
        std::string why;
        if (!pathBends(p, closed, c.kind == CurveKind::Polyline ? c.radius : 0.0f, bends, &why)) {
            c.radius = radiusBefore;
            return out.fail("Not changed: " + why);
        }
        setSpacePoints(c, p, closed);
        sketch.touch();
        doc.dirty = true;
        doc.selectedCurve = ci;
        out.say(said + "  " + c.measure());
        for (const Body& b : doc.scene.bodies())
            if (b.isRoute() && lowerOf(b.route.path) == lowerOf(c.name)) {
                out.say(b.name + " follows " + c.name + ": ROUTE UPDATE \"" + b.name + "\" refits it");
            }
        return out;
    }

    // PATH3D p1 p2 ... [SPLINE] [CLOSE] [BEND r] [CONSTRUCTION]
    std::vector<Vec3> p;
    bool spline = false, close = false, construction = false;
    float bend = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const bool q = i < quoted.size() && quoted[i];
        const std::string w = lowerOf(a[i]);
        if (!q && w == "spline") {
            spline = true;
        } else if (!q && (w == "close" || w == "closed")) {
            close = true;
        } else if (!q && (w == "construction" || w == "constr")) {
            construction = true;
        } else if (!q && w == "bend") {
            const auto r = i + 1 < a.size() ? parseLength(a[i + 1]) : std::nullopt;
            if (!r || *r < 0) return out.fail("BEND needs the bends' radius");
            bend = static_cast<float>(*r);
            ++i;
        } else if (!q && !a[i].empty() && a[i][0] == '@') {
            const auto v = parseVec3(a[i]);
            if (!v) return out.fail("Not a point: " + a[i] + "  (@dx,dy,dz: from the point before)");
            if (p.empty()) return out.fail("The first point can't be relative: x,y,z");
            p.push_back(p.back() + *v);
        } else if (const auto v = q ? std::nullopt : parseVec3(a[i])) {
            p.push_back(*v);
        } else {
            // A reference point by name (REFPOINT).
            const auto& refs = doc.state.refPoints;
            const auto it = std::find_if(refs.begin(), refs.end(), [&](const DatumPoint& r) { return lowerOf(r.name) == w; });
            if (it == refs.end()) return out.fail("Not a point: " + a[i] + "  (x,y,z, @dx,dy,dz or a reference point's name)");
            p.push_back(it->at);
        }
    }
    if (close && p.size() > 2 && length(p.front() - p.back()) < 1e-6f) p.pop_back();
    if (p.size() < (close ? 3u : 2u))
        return out.fail("PATH3D x,y,z x,y,z ... [SPLINE] [CLOSE] [BEND r]: " + std::string(close ? "three points or more for a closed path" : "two points or more"));
    if (spline && bend > 0) return out.fail("A spline is smooth: BEND rounds a 3D polyline's corners");
    std::vector<PathBend> bends;
    std::string why;
    if (!pathBends(p, close, spline ? 0.0f : bend, bends, &why)) return out.fail("Cannot make the path: " + why);
    SketchCurve c;
    c.kind = spline ? CurveKind::Spline : CurveKind::Polyline;
    c.plane = SketchPlane::Top;
    c.radius = spline ? 0.0f : bend;
    c.construction = construction;
    setSpacePoints(c, p, close);
    const SketchCurve& added = sketch.add(std::move(c));
    doc.selectedCurve = static_cast<int>(sketch.curves().size()) - 1;
    doc.selectedBody = doc.selectedImage = -1;
    doc.dirty = true;
    out.say("Added " + added.name + " through " + std::to_string(p.size()) + " points  " + added.measure());
    return out;
}

CommandOutput runRouteCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    auto& bodies = doc.scene.bodies();
    const std::string sub = a.empty() || (!quoted.empty() && quoted[0]) ? std::string() : lowerOf(a[0]);
    static const char* const kKinds[] = {"pipe", "tube", "rod", "bar", "cable", "wire", "hose"};
    auto isKind = [](const std::string& w) { return std::any_of(std::begin(kKinds), std::end(kKinds), [&](const char* k) { return w == k; }); };
    auto isOption = [](const std::string& w) { return w == "bend" || w == "material" || w == "name"; };
    // A route body by name (leading words), or the selected one.
    auto routeAt = [&](std::size_t& i) -> int {
        const std::string n = leadingName(a, quoted, i, [&](const std::string& s) { return bodyIndex(doc, s) >= 0; }, [&](const std::string& w) { return isKind(w) || isOption(w) || w == "export"; });
        const int bi = n.empty() ? doc.selectedBody : bodyIndex(doc, n);
        return bi >= 0 && bi < static_cast<int>(bodies.size()) && bodies[static_cast<std::size_t>(bi)].isRoute() ? bi : -1;
    };
    // The options after the size: BEND r, MATERIAL m, NAME n.
    struct Options {
        std::optional<float> bend;
        std::string material, name;
    };
    auto options = [&](std::size_t& i, std::vector<std::string>& sizeWords, Options& o) -> std::string {
        for (; i < a.size(); ++i) {
            const std::string w = lowerOf(a[i]);
            const bool q = i < quoted.size() && quoted[i];
            if (!q && w == "bend") {
                const auto r = i + 1 < a.size() ? parseLength(a[i + 1]) : std::nullopt;
                if (!r || !(*r > 0)) return "BEND needs the bends' centre-line radius";
                o.bend = static_cast<float>(*r);
                ++i;
            } else if (!q && w == "material" && i + 1 < a.size()) {
                o.material = a[++i];
                o.material[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(o.material[0])));
            } else if (!q && w == "name" && i + 1 < a.size()) {
                o.name = a[++i];
            } else {
                sizeWords.push_back(a[i]);
            }
        }
        return {};
    };
    auto join = [](const std::vector<std::string>& w) {
        std::string s;
        for (const std::string& x : w) s += (s.empty() ? "" : " ") + x;
        return s;
    };

    if (sub == "sizes") {
        const std::string kind = a.size() > 1 ? lowerOf(a[1]) : "pipe";
        const auto sizes = routeSizes(kind);
        if (sizes.empty()) return out.fail("ROUTE SIZES PIPE|TUBE|ROD|CABLE|HOSE");
        std::string line;
        for (const std::string& s : sizes) line += (line.empty() ? "" : ", ") + s;
        out.say(routeKindName(kind) + " sizes: " + line + (kind == "pipe" ? "  (any other: PIPE od x wall)" : ""));
        return out;
    }
    if (sub == "list") {
        int n = 0;
        double total = 0;
        for (const Body& b : bodies) {
            if (!b.isRoute()) continue;
            ++n;
            const RouteReport rep = routeReport(b.route);
            const auto spec = routeSpec(b.route.spec);
            total += rep.mass;
            std::string state = pathStatus(doc, b);
            if (b.route.key && b.route.key != meshKey(b.mesh)) state = "changed since it was made";
            out.say("  " + b.name + ": " + (spec ? spec->designation : b.route.spec) + " along " + b.route.path + ", " + lengthText(rep.length) +
                    (b.route.spline ? ", following a spline" : ", " + std::to_string(rep.bends.size()) + (rep.bends.size() == 1 ? " bend" : " bends") + " at R " + lengthText(b.route.bend)) +
                    ", " + num(rep.mass, 4) + " kg (" + b.route.material + ")" + (state.empty() ? "" : "  - " + state));
        }
        if (!n) out.say("No routes (ROUTE path PIPE DN20 makes one along a 3D path)");
        else out.say("Mass: " + num(total, 4) + " kg");
        return out;
    }
    if (sub == "bends") {
        // ROUTE BENDS [route] [EXPORT file.csv]
        std::size_t i = 1;
        const int bi = routeAt(i);
        if (bi < 0) return out.fail("ROUTE BENDS route [EXPORT file.csv]: which route?");
        const Body& b = bodies[static_cast<std::size_t>(bi)];
        if (b.route.spline) return out.fail(b.name + " follows a spline: it is bent to the curve, with no table of bends");
        const RouteReport rep = routeReport(b.route);
        if (!rep.why.empty()) return out.fail(b.name + ": " + rep.why);
        const auto spec = routeSpec(b.route.spec);
        out.say(b.name + ": " + (spec ? spec->designation : b.route.spec) + ", cut " + lengthText(rep.length) + ", " + std::to_string(rep.bends.size()) +
                (rep.bends.size() == 1 ? " bend" : " bends") + " at R " + lengthText(b.route.bend) + " (feed, rotate, bend):");
        for (std::size_t k = 0; k < rep.bends.size(); ++k) {
            const RouteBend& r = rep.bends[k];
            out.say("  " + std::to_string(k + 1) + ". feed " + lengthText(r.feed) + ", rotate " + num(r.rotation) + " deg, bend " + num(r.angle) + " deg  (at " + pointText(r.corner) + ")");
        }
        if (!b.route.closed) out.say("  then feed " + lengthText(rep.lastFeed));
        if (unquotedIs(a, quoted, i, "export")) {
            if (i + 1 >= a.size()) return out.fail("EXPORT needs a file name (.csv)");
            const std::string path = a[i + 1];
            std::ofstream f(path, std::ios::binary);
            if (!f) return out.fail("Cannot write " + path);
            f << "bend,feed_mm,rotation_deg,angle_deg,radius_mm,arc_mm,x_mm,y_mm,z_mm\n";
            for (std::size_t k = 0; k < rep.bends.size(); ++k) {
                const RouteBend& r = rep.bends[k];
                f << k + 1 << "," << num(r.feed, 7) << "," << num(r.rotation, 7) << "," << num(r.angle, 7) << "," << num(r.radius, 7) << "," << num(r.arc, 7) << ","
                  << num(r.corner.x, 7) << "," << num(r.corner.y, 7) << "," << num(r.corner.z, 7) << "\n";
            }
            if (!b.route.closed) f << "end," << num(rep.lastFeed, 7) << ",,,,,,,\n";
            out.say("Wrote the bends to " + path);
        }
        return out;
    }
    if (sub == "update") {
        // ROUTE UPDATE [route | ALL]
        std::size_t i = 1;
        std::vector<int> which;
        if (unquotedIs(a, quoted, 1, "all") || (a.size() == 1 && doc.selectedBody < 0)) {
            for (std::size_t k = 0; k < bodies.size(); ++k)
                if (bodies[k].isRoute()) which.push_back(static_cast<int>(k));
        } else {
            const int bi = routeAt(i);
            if (bi < 0) return out.fail("ROUTE UPDATE [route | ALL]: which route?");
            which.push_back(bi);
        }
        int updated = 0;
        for (const int bi : which) {
            Body& b = bodies[static_cast<std::size_t>(bi)];
            const std::string state = pathStatus(doc, b);
            if (state.empty()) continue;
            const int ci = curveIndex(doc, b.route.path);
            if (ci < 0) {
                out.say(b.name + ": " + state + " (left as it is)");
                continue;
            }
            if (b.route.key && b.route.key != meshKey(b.mesh)) {
                out.say(b.name + " was changed since it was made (cut, drilled...): left as it is");
                continue;
            }
            Body next = b;
            std::string why;
            if (!pathFromCurve(doc.scene.sketch().curves()[static_cast<std::size_t>(ci)], next.route, &why) || !remakeRoute(next, &why)) {
                out.say(b.name + ": " + why + " (left as it is)");
                continue;
            }
            b = std::move(next);
            ++updated;
            out.say("Refitted " + b.name + " to " + b.route.path + ": " + lengthText(routeReport(b.route).length));
        }
        if (updated) doc.scene.touchBodies(), doc.dirty = true;
        else if (out.lines.empty()) out.say(which.size() == 1 ? bodies[static_cast<std::size_t>(which[0])].name + " lies along its path" : "Every route lies along its path");
        return out;
    }
    if (sub == "edit") {
        // ROUTE EDIT route [size] [BEND r] [MATERIAL m]
        std::size_t i = 1;
        const int bi = routeAt(i);
        if (bi < 0) return out.fail("ROUTE EDIT route [PIPE DN25 | TUBE 12x1 | ...] [BEND r] [MATERIAL m]");
        Body& b = bodies[static_cast<std::size_t>(bi)];
        if (b.route.key && b.route.key != meshKey(b.mesh)) return out.fail(b.name + " was changed since it was made (cut, drilled...): it can't be remade");
        std::vector<std::string> sizeWords;
        Options o;
        if (const std::string w = options(i, sizeWords, o); !w.empty()) return out.fail(w);
        Body next = b;
        if (!sizeWords.empty()) {
            std::string why;
            const auto spec = routeSpec(join(sizeWords), &why);
            if (!spec) return out.fail(why);
            next.route.spec = spec->spec, next.route.od = spec->od, next.route.wall = spec->wall;
        }
        if (o.bend) next.route.bend = *o.bend;
        if (!o.material.empty()) next.route.material = o.material;
        if (!o.name.empty()) return out.fail("NAME names a new route; RENAME renames this one");
        std::string why;
        if (!remakeRoute(next, &why)) return out.fail("Cannot change " + b.name + ": " + why);
        next.offset = b.offset; // where it is (a route is made where its path is)
        b = std::move(next);
        doc.scene.touchBodies();
        doc.dirty = true;
        doc.selectedBody = bi;
        const auto spec = routeSpec(b.route.spec);
        out.say("Changed " + b.name + ": " + (spec ? spec->designation : b.route.spec) + (b.route.spline ? "" : ", bends at R " + lengthText(b.route.bend)) + ", " + b.route.material);
        return out;
    }

    // ROUTE [path] size [BEND r] [MATERIAL m] [NAME n]
    std::size_t i = 0;
    const std::string pathName = leadingName(a, quoted, i, [&](const std::string& n) { return curveIndex(doc, n) >= 0; }, [&](const std::string& w) { return isKind(w); });
    if (pathName.empty() && i < a.size() && !isKind(lowerOf(a[i]))) return out.fail("No sketch curve named " + a[i]);
    const int ci = pathName.empty() ? doc.selectedCurve : curveIndex(doc, pathName);
    if (!pathName.empty() && ci < 0) return out.fail("No sketch curve named " + pathName);
    if (ci < 0 || ci >= static_cast<int>(doc.scene.sketch().curves().size()))
        return out.fail("ROUTE path PIPE DN20 | TUBE 12x1 | ROD 10 | CABLE 8 | HOSE 19x3.5 [BEND r] [MATERIAL m]: along which path? (PATH3D makes one)");
    std::vector<std::string> sizeWords;
    Options o;
    if (const std::string w = options(i, sizeWords, o); !w.empty()) return out.fail(w);
    if (sizeWords.empty()) return out.fail("ROUTE " + pathName + " needs a size: PIPE DN20, TUBE 12x1, ROD 10, CABLE 8, HOSE 19x3.5 (ROUTE SIZES PIPE lists them)");
    std::string why;
    const auto spec = routeSpec(join(sizeWords), &why);
    if (!spec) return out.fail(why);
    const SketchCurve& c = doc.scene.sketch().curves()[static_cast<std::size_t>(ci)];
    Body::Route r;
    if (!pathFromCurve(c, r, &why)) return out.fail(why);
    r.spec = spec->spec, r.od = spec->od, r.wall = spec->wall;
    r.material = !o.material.empty() ? o.material : spec->flexible ? "Plastic" : "Steel";
    r.bend = r.spline ? 0.0f : o.bend ? *o.bend : c.spatial() && c.radius > 0 ? c.radius : spec->defaultBend();
    if (!o.name.empty() && bodyIndex(doc, o.name) >= 0) return out.fail("There is already a body named " + o.name);
    const bool flexible = spec->flexible;
    const Vec3 colour = spec->kind == "cable" ? Vec3{0.10f, 0.10f, 0.11f} : spec->kind == "hose" ? Vec3{0.16f, 0.17f, 0.20f} : Vec3{0.66f, 0.68f, 0.71f};
    Body b = meshBody(o.name.empty() ? newRouteName(doc, spec->kind) : o.name, MeshData{}, colour);
    if (!flexible) b.material.metallic = 0.8f, b.material.roughness = 0.38f;
    else b.material.metallic = 0.0f, b.material.roughness = 0.7f;
    b.route = r;
    if (!remakeRoute(b, &why)) return out.fail("Cannot route " + spec->designation + " along " + c.name + ": " + why);
    const RouteReport rep = routeReport(b.route);
    out.say("Added " + b.name + ": " + spec->designation + " along " + c.name + ", " + lengthText(rep.length) +
            (r.spline ? "" : ", " + std::to_string(rep.bends.size()) + (rep.bends.size() == 1 ? " bend" : " bends") + " at R " + lengthText(r.bend)) + ", " + num(rep.mass, 4) + " kg (" +
            r.material + ")" + (b.exact() ? ", exact" : ""));
    doc.scene.add(std::move(b));
    doc.selectedBody = static_cast<int>(bodies.size()) - 1;
    doc.selectedCurve = doc.selectedImage = -1;
    doc.scene.touchBodies();
    doc.dirty = true;
    return out;
}

} // namespace sicdm

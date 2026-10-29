// SECTION: the live section view (SectionView.hpp).

#include "core/SectionView.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Extrude.hpp"
#include "core/Features.hpp"
#include "core/Sculpt.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <limits>

namespace sicdm {

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string num(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.4g", std::fabs(v) < 5e-10 ? 0.0 : v);
    return b;
}

// The plane's frame: through the point of it nearest the origin, its normal the section's.
PlaneFrame frameOf(const SectionView& s) {
    PlaneFrame f;
    const Vec3 n = s.normal;
    makePlaneFrame(n * static_cast<float>(s.offset), n, std::fabs(n.z) > 0.9f ? Vec3{1, 0, 0} : Vec3{0, 0, 1}, f);
    return f;
}

// A body's mesh where it is drawn: in the world, and moved by its transform when it has one.
MeshData placed(const Body& b, std::size_t i, const std::vector<Mat4>* transforms) {
    MeshData m = worldMesh(b);
    if (transforms && i < transforms->size())
        for (Vertex& v : m.vertices) v.position = (*transforms)[i].transformPoint(v.position);
    return m;
}

bool straddles(const MeshData& m, const SectionView& s) {
    double lo = std::numeric_limits<double>::max(), hi = -lo;
    for (const Vertex& v : m.vertices) {
        const double d = s.distance(v.position);
        lo = std::min(lo, d), hi = std::max(hi, d);
    }
    return lo < 0 && hi > 0;
}

// The section's loops of each cut body (closed, visible, crossing the plane).
struct CutLoops {
    int body = -1;
    std::vector<std::vector<Vec2>> loops;
};
std::vector<CutLoops> cutLoops(const Scene& scene, const SectionView& s, const std::vector<Mat4>* transforms) {
    std::vector<CutLoops> out;
    const PlaneFrame f = frameOf(s);
    const auto& bodies = scene.bodies();
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const Body& b = bodies[i];
        if (!b.visible || b.mesh.indices.empty() || !s.cuts(b)) continue;
        const MeshData m = placed(b, i, transforms);
        if (!straddles(m, s) || !watertight(m)) continue;
        CutLoops c;
        c.body = static_cast<int>(i);
        c.loops = planeSection(m, f);
        if (!c.loops.empty()) out.push_back(std::move(c));
    }
    return out;
}

// The loops of the scene as it stands, kept while nothing changes (picking asks often).
const std::vector<CutLoops>& cachedLoops(const Scene& scene, const SectionView& s, const std::vector<Mat4>* transforms) {
    thread_local struct {
        const Scene* scene = nullptr;
        std::uint64_t revision = ~std::uint64_t{0};
        SectionView view;
        std::vector<CutLoops> loops;
    } cache;
    thread_local std::vector<CutLoops> uncached;
    if (transforms) return uncached = cutLoops(scene, s, transforms); // moving parts: as they are now
    if (cache.scene != &scene || cache.revision != scene.revision() || !(cache.view == s)) {
        cache.loops = cutLoops(scene, s, nullptr);
        cache.scene = &scene, cache.revision = scene.revision(), cache.view = s;
    }
    return cache.loops;
}

bool insideLoops(const std::vector<std::vector<Vec2>>& loops, const Vec2& q) {
    bool in = false;
    for (const auto& l : loops)
        for (std::size_t k = 0, j = l.size() - 1; k < l.size(); j = k++)
            if ((l[k].y > q.y) != (l[j].y > q.y) && q.x < (l[j].x - l[k].x) * (q.y - l[k].y) / (l[j].y - l[k].y) + l[k].x) in = !in;
    return in;
}

const char* axisName(const Vec3& n) {
    if (std::fabs(std::fabs(n.x) - 1) < 1e-6f) return "x";
    if (std::fabs(std::fabs(n.y) - 1) < 1e-6f) return "y";
    if (std::fabs(std::fabs(n.z) - 1) < 1e-6f) return "z";
    return nullptr;
}

// The middle of what is shown (where a new plane goes by default).
Vec3 sceneMiddle(const Scene& scene) {
    AABB box;
    for (const Body& b : scene.bodies())
        if (b.visible && !b.mesh.indices.empty()) box.expand(b.bounds());
    return box.valid() ? (box.min + box.max) * 0.5f : Vec3{};
}

} // namespace

bool SectionView::cuts(const Body& b) const {
    if (only.empty()) return true;
    for (const std::string& n : only)
        if (b.name == n || (b.name.size() > n.size() && b.name.compare(0, n.size(), n) == 0 && b.name[n.size()] == ':')) return true;
    return false;
}

Json sectionToJson(const SectionView& s) {
    Json j = Json::object();
    j.set("on", s.on);
    j.set("normal", Json::array().push(s.normal.x).push(s.normal.y).push(s.normal.z));
    j.set("offset", s.offset);
    j.set("cap", s.cap);
    if (!s.only.empty()) {
        Json o = Json::array();
        for (const std::string& n : s.only) o.push(n);
        j.set("only", o);
    }
    return j;
}

SectionView sectionFromJson(const Json& j) {
    SectionView s;
    if (!j.isObject()) return s;
    s.on = j["on"].asBool(false);
    if (j["normal"].size() == 3) {
        const Vec3 n{j["normal"][0].asFloat(), j["normal"][1].asFloat(), j["normal"][2].asFloat()};
        if (length(n) > 1e-6f) s.normal = normalize(n);
    }
    s.offset = j["offset"].asNumber(0);
    if (!std::isfinite(s.offset)) s.offset = 0;
    s.cap = j["cap"].asBool(true);
    for (std::size_t i = 0; i < j["only"].size(); ++i)
        if (j["only"][i].isString()) s.only.push_back(j["only"][i].asString());
    return s;
}

std::vector<SectionCut> sectionCuts(const Scene& scene, const SectionView& s) {
    std::vector<SectionCut> out;
    const PlaneFrame f = frameOf(s);
    for (const CutLoops& c : cutLoops(scene, s, nullptr)) {
        SectionCut cut;
        cut.body = scene.bodies()[static_cast<std::size_t>(c.body)].name;
        cut.loops = static_cast<int>(c.loops.size());
        double a = 0, cx = 0, cy = 0;
        for (const auto& l : c.loops)
            for (std::size_t k = 0, j = l.size() - 1; k < l.size(); j = k++) {
                const double cr = static_cast<double>(l[j].x) * l[k].y - static_cast<double>(l[k].x) * l[j].y;
                a += cr;
                cx += (static_cast<double>(l[j].x) + l[k].x) * cr;
                cy += (static_cast<double>(l[j].y) + l[k].y) * cr;
            }
        if (std::fabs(a) < 1e-12) continue;
        cut.area = std::fabs(a) * 0.5;
        cut.centroid = f.toWorld({static_cast<float>(cx / (3 * a)), static_cast<float>(cy / (3 * a))});
        out.push_back(cut);
    }
    return out;
}

std::optional<RayHit> sectionRaycast(const Scene& scene, const SectionView& s, const Ray& ray, const std::vector<Mat4>* transforms) {
    std::optional<RayHit> hit = scene.raycast(ray, transforms);
    if (!s.on) return hit;
    const auto& bodies = scene.bodies();
    const double eps = 1e-4;
    auto takenAway = [&](const RayHit& h) { return h.body >= 0 && s.cuts(bodies[static_cast<std::size_t>(h.body)]) && s.distance(h.point) > eps; };
    // Where the ray crosses the plane.
    const double along = dot(s.normal, ray.dir);
    const double t0 = std::fabs(along) > 1e-9 ? -s.distance(ray.origin) / along : -1;
    if (hit && takenAway(*hit)) {
        hit.reset();
        if (t0 > 0) { // beyond the plane, on the side kept
            const float start = static_cast<float>(t0) + 1e-4f;
            Ray beyond{ray.at(start), ray.dir};
            if (auto h = scene.raycast(beyond, transforms); h && !takenAway(*h)) {
                h->t += start;
                hit = h;
            }
        }
    }
    // A cut face there, nearer than what the ray hits?
    if (s.cap && t0 > 0 && (!hit || t0 < hit->t)) {
        const Vec3 p = ray.at(static_cast<float>(t0));
        const PlaneFrame f = frameOf(s);
        for (const CutLoops& c : cachedLoops(scene, s, transforms))
            if (insideLoops(c.loops, f.toPlane(p))) return RayHit{c.body, static_cast<float>(t0), p};
    }
    return hit;
}

std::string describeSection(const SectionView& s) {
    if (const char* a = axisName(s.normal)) {
        const float sign = s.normal.x + s.normal.y + s.normal.z > 0 ? 1.0f : -1.0f;
        return std::string(a) + " = " + num(s.offset * sign) + ", keeping " + a + (sign > 0 ? " < " : " > ") + num(s.offset * sign);
    }
    return "the plane square to (" + num(s.normal.x) + ", " + num(s.normal.y) + ", " + num(s.normal.z) + ") " + num(s.offset) +
           " mm from the origin, keeping the side away from its normal";
}

CommandOutput runSectionCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    SectionView& s = doc.state.section;
    SectionView next = s;
    auto word = [&](std::size_t i, const char* w) { return i < a.size() && !(i < quoted.size() && quoted[i]) && lowerOf(a[i]) == w; };
    bool placed = false; // a new plane (it turns the section on)
    std::optional<Vec3> through;
    std::optional<double> at;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const std::string w = i < quoted.size() && quoted[i] ? std::string() : lowerOf(a[i]);
        if (w == "on" || w == "show") next.on = true;
        else if (w == "off" || w == "hide") next.on = false;
        else if (w == "list" || w == "info") {
        } else if (w == "x" || w == "y" || w == "z" || w == "-x" || w == "-y" || w == "-z") {
            const bool neg = w[0] == '-';
            const char axis = w.back();
            next.normal = Vec3{axis == 'x' ? 1.0f : 0.0f, axis == 'y' ? 1.0f : 0.0f, axis == 'z' ? 1.0f : 0.0f} * (neg ? -1.0f : 1.0f);
            placed = true;
        } else if (w == "plane" && i + 1 < a.size()) {
            const auto n = parseVec3(a[++i]);
            if (!n || length(*n) < 1e-9f) return out.fail("PLANE needs its normal nx,ny,nz");
            next.normal = normalize(*n);
            placed = true;
        } else if (w == "at" && i + 1 < a.size()) {
            ++i;
            if (a[i].find(',') != std::string::npos) {
                through = parseVec3(a[i]);
                if (!through) return out.fail("AT needs a point x,y,z or a distance along the plane's axis");
            } else {
                at = parseLength(a[i]);
                if (!at) return out.fail("AT needs a distance along the plane's axis, or a point x,y,z");
            }
            next.on = true;
        } else if (w == "offset" && i + 1 < a.size()) {
            at = parseLength(a[++i]);
            if (!at) return out.fail("OFFSET needs a distance");
            next.on = true;
        } else if (w == "move" && i + 1 < a.size()) {
            const auto d = parseLength(a[++i]);
            if (!d) return out.fail("MOVE needs a distance");
            next.offset += *d;
            next.on = true;
        } else if (w == "flip") {
            next.normal = next.normal * -1.0f;
            next.offset = -next.offset;
            next.on = true;
        } else if (w == "cap" && i + 1 < a.size() && (word(i + 1, "on") || word(i + 1, "off"))) {
            next.cap = word(++i, "on");
        } else if (w == "all") {
            next.only.clear();
        } else if (w == "only") {
            next.only.clear();
            for (++i; i < a.size(); ++i) {
                const std::string n = a[i];
                if (std::none_of(doc.scene.bodies().begin(), doc.scene.bodies().end(), [&](const Body& b) { return b.name == n || b.name.rfind(n + ":", 0) == 0; }))
                    return out.fail("No body " + n);
                next.only.push_back(n);
            }
            if (next.only.empty()) return out.fail("ONLY needs the bodies to cut");
            next.on = true;
        } else {
            return out.fail("SECTION [X|Y|Z|-X|-Y|-Z|PLANE nx,ny,nz] [AT d | AT x,y,z] | MOVE d | FLIP | ON | OFF | CAP ON|OFF | ONLY body... | ALL");
        }
    }
    // Where the plane goes: through the point, at the distance along its axis, or (a new
    // plane) through the middle of the model.
    if (through) next.offset = dot(next.normal, *through);
    else if (at) {
        const float sign = axisName(next.normal) && next.normal.x + next.normal.y + next.normal.z < 0 ? -1.0f : 1.0f;
        next.offset = *at * sign;
    } else if (placed) next.offset = dot(next.normal, sceneMiddle(doc.scene));
    if (placed) next.on = true;
    s = next;
    if (!s.on) {
        out.say("Section view off (the plane: " + describeSection(s) + "; SECTION ON shows it)");
        return out;
    }
    out.say("Section view on " + describeSection(s) + (s.cap ? "" : " (cut faces not filled)") + (s.only.empty() ? "" : " (cutting only the chosen bodies)"));
    const auto cuts = sectionCuts(doc.scene, s);
    if (cuts.empty()) out.say("  The plane cuts no solid");
    for (const SectionCut& c : cuts)
        out.say("  " + c.body + ": section area " + num(c.area) + " mm2, centroid (" + num(c.centroid.x) + ", " + num(c.centroid.y) + ", " + num(c.centroid.z) +
                ")" + (c.loops > 1 ? ", " + std::to_string(c.loops) + " loops" : ""));
    return out;
}

} // namespace sicdm

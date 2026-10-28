// Exploded views of an assembly (Assembly.hpp): which bodies a step moves, how far each body
// is drawn from where it sits, and steps made automatically.

#include "core/Assembly.hpp"
#include "core/Scene.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <map>

namespace sicdm {

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

// Adds the bodies attached to any in `in` (a hinge pin on its part, armour on its wearer), at
// any depth.
void withAttached(const Scene& scene, std::vector<std::size_t>& in) {
    const auto& bodies = scene.bodies();
    std::vector<char> on(bodies.size(), 0);
    for (std::size_t i : in) on[i] = 1;
    for (bool grew = true; grew;) {
        grew = false;
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            if (on[i] || bodies[i].attachedTo.empty()) continue;
            for (std::size_t j = 0; j < bodies.size(); ++j)
                if (on[j] && bodies[j].name == bodies[i].attachedTo) {
                    on[i] = 1, grew = true;
                    in.push_back(i);
                    break;
                }
        }
    }
}

// The body a body is attached to at the root (itself when attached to nothing that exists).
std::size_t attachRoot(const Scene& scene, std::size_t i) {
    const auto& bodies = scene.bodies();
    for (std::size_t hops = 0; hops < bodies.size() && !bodies[i].attachedTo.empty(); ++hops) {
        std::size_t next = bodies.size();
        for (std::size_t j = 0; j < bodies.size(); ++j)
            if (bodies[j].name == bodies[i].attachedTo) next = j;
        if (next == bodies.size()) break;
        i = next;
    }
    return i;
}

std::string num(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.6g", v);
    return b;
}

} // namespace

std::vector<std::size_t> explodeBodies(const Scene& scene, const std::string& part) {
    const auto& bodies = scene.bodies();
    const std::string want = lowerOf(part);
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < bodies.size() && out.empty(); ++i)
        if (lowerOf(bodies[i].name) == want) out.push_back(i);
    if (out.empty())
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (!bodies[i].component.empty() && lowerOf(bodies[i].component) == want) out.push_back(i);
    if (out.empty())
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            const std::string a = lowerOf(bodies[i].assembly);
            if (!a.empty() && (a == want || a.rfind(want + "/", 0) == 0)) out.push_back(i);
        }
    if (!out.empty()) withAttached(scene, out);
    return out;
}

std::vector<DVec3> explodeShifts(const Scene& scene, const ExplodeState& st, double t) {
    std::vector<DVec3> shift(scene.bodies().size(), DVec3{0, 0, 0});
    const double n = static_cast<double>(st.steps.size());
    for (std::size_t k = 0; k < st.steps.size(); ++k) {
        const ExplodeStep& s = st.steps[k];
        const double f = std::clamp(t * n - static_cast<double>(k), 0.0, 1.0);
        if (!(f > 0)) continue;
        std::vector<char> moved(shift.size(), 0);
        for (const std::string& p : s.parts)
            for (std::size_t i : explodeBodies(scene, p))
                if (!moved[i]) moved[i] = 1, shift[i] = shift[i] + s.dir * (s.distance * f);
    }
    return shift;
}

std::vector<ExplodeStep> autoExplode(const Scene& scene, const std::vector<std::size_t>& only, int axis, double factor) {
    const auto& bodies = scene.bodies();
    std::vector<std::size_t> chosen = only;
    if (chosen.empty())
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (bodies[i].visible && !bodies[i].mesh.indices.empty()) chosen.push_back(i);
    // The parts: a component instance moves as one, a body attached to another goes with it.
    struct Unit {
        std::string name;
        AABB box;
        double volume = 0;
        DVec3 shift{0, 0, 0};
    };
    std::vector<Unit> units;
    std::map<std::string, std::size_t> unitOf;
    std::vector<char> isChosen(bodies.size(), 0);
    for (std::size_t i : chosen) isChosen[i] = 1;
    for (std::size_t i : chosen) {
        const std::size_t root = attachRoot(scene, i);
        if (root != i && isChosen[root]) continue; // it goes with what it is attached to
        const Body& b = bodies[i];
        const std::string key = b.component.empty() ? "b:" + b.name : "c:" + b.component;
        const auto [it, added] = unitOf.try_emplace(key, units.size());
        if (added) units.push_back({b.component.empty() ? b.name : b.component, AABB{}, 0, {}});
        units[it->second].box.expand(b.bounds());
    }
    AABB all;
    for (Unit& u : units) {
        all.expand(u.box);
        const Vec3 e = u.box.valid() ? u.box.max - u.box.min : Vec3{};
        u.volume = static_cast<double>(e.x) * e.y * e.z;
    }
    if (units.size() < 2 || !all.valid()) return {};
    const DVec3 mid = DVec3(all.center());
    const double diag = static_cast<double>(length(all.max - all.min));
    if (axis < 0) {
        // Out from the middle, each part as far again as it is from it.
        for (Unit& u : units) {
            const DVec3 v = DVec3(u.box.center()) - mid;
            if (dlength(v) > 1e-6 * std::max(diag, 1.0)) u.shift = v * factor;
        }
    } else {
        // Stacked apart along the axis: the largest part stays; the others go to the side of it
        // their middles are on, each clear (by a gap) of whatever is between it and the
        // largest part and in its way (its outline seen along the axis overlaps).
        auto along = [&](const Vec3& p) { return static_cast<double>(axis == 0 ? p.x : axis == 1 ? p.y : p.z); };
        auto across = [&](const Unit& a, const Unit& b) {
            const int p = (axis + 1) % 3, q = (axis + 2) % 3;
            auto c = [](const Vec3& v, int k) { return static_cast<double>(k == 0 ? v.x : k == 1 ? v.y : v.z); };
            const double tol = 1e-4 * std::max(diag, 1.0);
            return c(a.box.min, p) < c(b.box.max, p) - tol && c(a.box.max, p) > c(b.box.min, p) + tol && c(a.box.min, q) < c(b.box.max, q) - tol &&
                   c(a.box.max, q) > c(b.box.min, q) + tol;
        };
        const double gap = factor * 0.15 * std::max(along(all.max) - along(all.min), 1e-3);
        std::size_t anchor = 0;
        for (std::size_t k = 1; k < units.size(); ++k)
            if (units[k].volume > units[anchor].volume) anchor = k;
        const double c0 = along(units[anchor].box.center());
        std::vector<std::size_t> up, down;
        for (std::size_t k = 0; k < units.size(); ++k)
            if (k != anchor) (along(units[k].box.center()) >= c0 - 1e-9 ? up : down).push_back(k);
        auto mid1 = [&](std::size_t k) { return along(units[k].box.center()); };
        std::stable_sort(up.begin(), up.end(), [&](std::size_t a, std::size_t b) { return mid1(a) < mid1(b); });
        std::stable_sort(down.begin(), down.end(), [&](std::size_t a, std::size_t b) { return mid1(a) > mid1(b); });
        DVec3 dir{0, 0, 0};
        (axis == 0 ? dir.x : axis == 1 ? dir.y : dir.z) = 1;
        std::vector<std::size_t> placed{anchor};
        std::vector<double> moveBy(units.size(), 0);
        for (std::size_t k : up) {
            double s = 0;
            for (std::size_t p : placed)
                if (across(units[k], units[p])) s = std::max(s, along(units[p].box.max) + moveBy[p] + gap - along(units[k].box.min));
            moveBy[k] = s;
            placed.push_back(k);
        }
        for (std::size_t k : down) {
            double s = 0;
            for (std::size_t p : placed)
                if (across(units[k], units[p])) s = std::min(s, along(units[p].box.min) + moveBy[p] - gap - along(units[k].box.max));
            moveBy[k] = s;
            placed.push_back(k);
        }
        for (std::size_t k = 0; k < units.size(); ++k) units[k].shift = dir * moveBy[k];
    }
    // One step a distance and way: parts moved alike go together. The farthest come off first.
    std::vector<ExplodeStep> steps;
    for (const Unit& u : units) {
        const double d = dlength(u.shift);
        if (!(d > 1e-9)) continue;
        const DVec3 dir = u.shift * (1.0 / d);
        auto same = std::find_if(steps.begin(), steps.end(), [&](const ExplodeStep& s) { return std::fabs(s.distance - d) < 1e-6 * std::max(d, 1.0) && dlength(s.dir - dir) < 1e-9; });
        if (same != steps.end()) same->parts.push_back(u.name);
        else steps.push_back({{u.name}, dir, d});
    }
    std::stable_sort(steps.begin(), steps.end(), [](const ExplodeStep& a, const ExplodeStep& b) { return a.distance > b.distance; });
    return steps;
}

std::vector<ExplodeTrail> explodeTrails(const Scene& scene, const ExplodeState& st, double t, const std::function<bool(std::size_t)>& include) {
    std::vector<ExplodeTrail> out;
    const auto& bodies = scene.bodies();
    std::vector<DVec3> shift(bodies.size(), DVec3{0, 0, 0});
    const double n = static_cast<double>(st.steps.size());
    for (std::size_t k = 0; k < st.steps.size(); ++k) {
        const ExplodeStep& s = st.steps[k];
        const double f = std::clamp(t * n - static_cast<double>(k), 0.0, 1.0);
        if (!(f > 0)) break; // the steps take turns: none after this has started
        const DVec3 move = s.dir * (s.distance * f);
        std::vector<char> moved(bodies.size(), 0);
        for (const std::string& p : s.parts) {
            AABB box;
            std::size_t one = bodies.size();
            const std::vector<std::size_t> idx = explodeBodies(scene, p);
            for (std::size_t i : idx)
                if (include(i)) box.expand(bodies[i].bounds()), one = std::min(one, i);
            if (one < bodies.size() && box.valid()) {
                const DVec3 from = DVec3(box.center()) + shift[one];
                out.push_back({from, from + move});
            }
            for (std::size_t i : idx) moved[i] = 1;
        }
        for (std::size_t i = 0; i < bodies.size(); ++i)
            if (moved[i]) shift[i] = shift[i] + move;
    }
    return out;
}

std::string describeExplodeStep(const ExplodeStep& s) {
    std::string parts;
    for (const std::string& p : s.parts) parts += (parts.empty() ? "" : ", ") + p;
    const char* axis = nullptr;
    if (std::fabs(s.dir.x) > 1 - 1e-9) axis = s.dir.x > 0 ? "+X" : "-X";
    else if (std::fabs(s.dir.y) > 1 - 1e-9) axis = s.dir.y > 0 ? "+Y" : "-Y";
    else if (std::fabs(s.dir.z) > 1 - 1e-9) axis = s.dir.z > 0 ? "+Z" : "-Z";
    const std::string way = axis ? std::string(axis) : "(" + num(s.dir.x) + ", " + num(s.dir.y) + ", " + num(s.dir.z) + ")";
    return parts + " " + num(s.distance) + " mm along " + way;
}

} // namespace sicdm

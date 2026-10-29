// More than one body selected (see Selection.hpp).

#include "core/Selection.hpp"

#include "core/Scene.hpp"
#include "core/Sketch.hpp"

#include <algorithm>
#include <unordered_map>

namespace sicdm {

namespace {

// What bodies and curves share: a list of names, the selection kept by name.
template <class Items>
std::vector<int> selectedOf(const Items& items, int active, const BodySelection& sel) {
    std::vector<int> out;
    if (active < 0 || active >= static_cast<int>(items.size())) return out;
    out.push_back(active);
    if (sel.others.empty() || items[static_cast<std::size_t>(active)].name != sel.anchor) return out;
    // Names to items (the first of a name, as commands find them).
    std::unordered_map<std::string, int> index;
    index.reserve(items.size());
    for (std::size_t i = 0; i < items.size(); ++i) index.emplace(items[i].name, static_cast<int>(i));
    for (const std::string& n : sel.others)
        if (const auto it = index.find(n); it != index.end() && std::find(out.begin(), out.end(), it->second) == out.end()) out.push_back(it->second);
    return out;
}

template <class Items>
void selectOf(const Items& items, int& active, BodySelection& sel, const std::vector<int>& list) {
    sel = BodySelection{};
    active = -1;
    for (int b : list) {
        if (b < 0 || b >= static_cast<int>(items.size())) continue;
        const std::string& n = items[static_cast<std::size_t>(b)].name;
        if (active < 0) {
            active = b;
            sel.anchor = n;
        } else if (b != active && std::find(sel.others.begin(), sel.others.end(), n) == sel.others.end()) {
            sel.others.push_back(n);
        }
    }
}

template <class Items>
void toggleOf(const Items& items, int& active, BodySelection& sel, int item) {
    std::vector<int> now = selectedOf(items, active, sel);
    // Newest first: the one added last is active, and when it is taken out the one added
    // before it is.
    if (const auto it = std::find(now.begin(), now.end(), item); it != now.end()) now.erase(it);
    else now.insert(now.begin(), item);
    selectOf(items, active, sel, now);
}

template <class Items>
std::string freeName(const Items& items, const std::string& base, const std::vector<std::string>& taken, bool keepIfFree) {
    auto used = [&](const std::string& n) {
        if (std::find(taken.begin(), taken.end(), n) != taken.end()) return true;
        for (const auto& it : items)
            if (it.name == n) return true;
        return false;
    };
    if (keepIfFree && !used(base)) return base;
    // A copy of a copy is numbered from the original: "Block copy 2", not "Block copy copy".
    std::string root = base;
    if (const std::size_t at = root.rfind(" copy"); at != std::string::npos && at > 0) {
        const std::string tail = root.substr(at + 5);
        if (tail.empty() || (tail.size() > 1 && tail[0] == ' ' && std::all_of(tail.begin() + 1, tail.end(), [](char c) { return c >= '0' && c <= '9'; })))
            root = root.substr(0, at);
    }
    if (!used(root + " copy")) return root + " copy";
    for (int k = 2;; ++k)
        if (const std::string n = root + " copy " + std::to_string(k); !used(n)) return n;
}

} // namespace

std::vector<int> selectedBodies(const Scene& scene, int active, const BodySelection& sel) { return selectedOf(scene.bodies(), active, sel); }

void selectBodies(const Scene& scene, int& active, BodySelection& sel, const std::vector<int>& list) { selectOf(scene.bodies(), active, sel, list); }

void toggleBody(const Scene& scene, int& active, BodySelection& sel, int body) { toggleOf(scene.bodies(), active, sel, body); }

std::vector<Body>& bodyClipboard() {
    static std::vector<Body> clipboard;
    return clipboard;
}

std::string copyName(const Scene& scene, const std::string& base, const std::vector<std::string>& taken, bool keepIfFree) {
    return freeName(scene.bodies(), base, taken, keepIfFree);
}

std::vector<int> selectedCurves(const Sketch& sketch, int active, const BodySelection& sel) { return selectedOf(sketch.curves(), active, sel); }

void selectCurves(const Sketch& sketch, int& active, BodySelection& sel, const std::vector<int>& list) { selectOf(sketch.curves(), active, sel, list); }

void toggleCurve(const Sketch& sketch, int& active, BodySelection& sel, int curve) { toggleOf(sketch.curves(), active, sel, curve); }

std::vector<SketchCurve>& curveClipboard() {
    static std::vector<SketchCurve> clipboard;
    return clipboard;
}

std::string copyCurveName(const Sketch& sketch, const std::string& base, const std::vector<std::string>& taken, bool keepIfFree) {
    return freeName(sketch.curves(), base, taken, keepIfFree);
}

} // namespace sicdm

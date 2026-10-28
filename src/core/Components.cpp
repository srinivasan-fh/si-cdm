#include "core/Components.hpp"

#include "core/Brep.hpp"
#include "core/FileIO.hpp"
#include "core/Scene.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <functional>
#include <map>
#include <set>

namespace sicdm {

namespace fs = std::filesystem;

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string canonical(const std::string& file) {
    std::error_code ec;
    const fs::path p = fs::weakly_canonical(fs::absolute(file, ec), ec);
    return (ec ? fs::absolute(file).lexically_normal() : p).string();
}

// Whether two names are the one file: by the file itself where both exist (however they are
// spelled: links, letter case on file systems that ignore it), else by their canonical names.
bool sameFile(const std::string& a, const std::string& b) {
    if (a == b) return true;
    std::error_code ec;
    if (fs::exists(a, ec) && fs::exists(b, ec)) {
        const bool same = fs::equivalent(a, b, ec);
        if (!ec) return same;
    }
    return canonical(a) == canonical(b);
}

bool anySameFile(const std::set<std::string>& files, const std::string& f) {
    return std::any_of(files.begin(), files.end(), [&](const std::string& g) { return sameFile(g, f); });
}

// The instance's placement read off one of its bodies: its pose here relative to its pose in
// the part file (rotation, then where the part's origin went).
Placement instancePlacement(const Body& b) {
    const Placement inFile = eulerPlacement(decimal(b.link.partRotation)), here = eulerPlacement(decimal(b.rotation));
    Placement m = inFile.inverse().then(here);
    m.t = {0, 0, 0};
    m.t = decimal(b.offset) - m.point(decimal(b.link.partOffset));
    return m;
}

// Puts a body (as it is in its part file) where the placement takes it.
void place(Body& b, const Placement& m) {
    Placement turn = m;
    turn.t = {0, 0, 0};
    const DVec3 offset = m.point(decimal(b.offset));
    const Placement r = eulerPlacement(decimal(b.rotation)).then(turn);
    b.setRotation(eulerDegrees(r).f());
    b.offset = offset.f();
}

struct PartFile {
    Scene scene;
    ProjectState state;
};

int reloadImpl(Scene& scene, const std::string& name, bool force, std::vector<std::string>& report, std::set<std::string>& visiting);

// Reads a part file, its own linked parts brought up to date first (a sub-assembly). A file
// that links back to one being read is refused (it would never end).
bool readPart(const std::string& file, PartFile& part, std::string* why, std::set<std::string>& visiting) {
    const std::string key = canonical(file);
    if (anySameFile(visiting, key)) {
        if (why) *why = fs::path(file).filename().string() + " links back to itself (through its parts)";
        return false;
    }
    std::string err, warn;
    if (!loadProject(part.scene, part.state, file, &err, &warn)) {
        if (why) *why = err;
        return false;
    }
    visiting.insert(key);
    std::vector<std::string> nested;
    reloadImpl(part.scene, "", false, nested, visiting);
    visiting.erase(key);
    return true;
}

// Whether a scene links to `target`, at any depth (its parts' parts...).
bool reaches(const Scene& scene, const std::string& target, std::set<std::string>& seen) {
    for (const Body& b : scene.bodies()) {
        if (!b.linked()) continue;
        const std::string f = canonical(b.link.file);
        if (sameFile(f, target)) return true;
        if (anySameFile(seen, f)) continue;
        seen.insert(f);
        Scene s;
        ProjectState st;
        std::string err, warn;
        if (loadProject(s, st, f, &err, &warn) && reaches(s, target, seen)) return true;
    }
    return false;
}

bool nameTaken(const Scene& scene, const std::string& n) {
    const std::string l = lowerOf(n);
    for (const Body& b : scene.bodies())
        if (lowerOf(b.name) == l || lowerOf(b.component) == l) return true;
    return false;
}

std::string bodyName(const std::string& instance, const std::string& partBody, std::size_t count) {
    return count == 1 ? instance : instance + ":" + partBody;
}

// The part's bodies, placed (each where `placementOf` its name in the part file says), as
// instance `name`.
std::vector<std::string> addInstance(Scene& scene, const PartFile& part, const std::string& file, const std::string& name,
                                     const std::function<Placement(const std::string&)>& placementOf) {
    std::vector<std::string> made;
    const auto& src = part.scene.bodies();
    std::map<std::string, std::string> renamed;
    for (const Body& pb : src) renamed[pb.name] = bodyName(name, pb.name, src.size());
    const auto number = part.state.product.properties.find("NUMBER");
    const auto description = part.state.product.properties.find("DESCRIPTION");
    const std::uint64_t stamp = fileStamp(file);
    for (const Body& pb : src) {
        Body b = pb;
        b.name = renamed[pb.name];
        b.component = name;
        b.assembly = src.size() == 1 ? pb.assembly : (pb.assembly.empty() ? name : name + "/" + pb.assembly);
        b.attachedTo = renamed.count(pb.attachedTo) ? renamed[pb.attachedTo] : std::string();
        // The part's number and description go with it into the parts list.
        if (!b.properties.count("NUMBER") && number != part.state.product.properties.end()) b.properties["NUMBER"] = number->second;
        if (!b.properties.count("DESCRIPTION") && description != part.state.product.properties.end()) b.properties["DESCRIPTION"] = description->second;
        b.link.file = file;
        b.link.body = pb.name;
        b.link.partOffset = pb.offset;
        b.link.partRotation = pb.rotation;
        b.link.vertices = pb.mesh.vertices.size();
        b.link.stamp = stamp;
        place(b, placementOf(pb.name));
        made.push_back(b.name);
        scene.add(std::move(b));
    }
    return made;
}

std::vector<std::size_t> instanceBodies(const Scene& scene, const std::string& name) {
    std::vector<std::size_t> out;
    const std::string l = lowerOf(name);
    for (std::size_t i = 0; i < scene.bodies().size(); ++i)
        if (scene.bodies()[i].linked() && lowerOf(scene.bodies()[i].component) == l) out.push_back(i);
    return out;
}

int reloadImpl(Scene& scene, const std::string& name, bool force, std::vector<std::string>& report, std::set<std::string>& visiting) {
    // The instances (in the order they first appear) and their files.
    std::vector<std::pair<std::string, std::string>> instances;
    for (const Body& b : scene.bodies())
        if (b.linked() && (name.empty() || lowerOf(b.component) == lowerOf(name)) &&
            std::none_of(instances.begin(), instances.end(), [&](const auto& x) { return x.first == b.component; }))
            instances.push_back({b.component, b.link.file});
    int reloaded = 0;
    for (const auto& [instance, file] : instances) {
        std::error_code ec;
        if (!fs::exists(file, ec)) {
            report.push_back(instance + ": " + file + " is missing (its last shape is kept)");
            continue;
        }
        std::vector<std::size_t> idx = instanceBodies(scene, instance);
        if (idx.empty()) continue;
        const std::uint64_t stamp = fileStamp(file);
        bool changed = force;
        for (std::size_t i : idx) changed = changed || scene.bodies()[i].link.stamp != stamp;
        if (!changed) continue;
        PartFile part;
        std::string why;
        if (!readPart(file, part, &why, visiting)) {
            report.push_back(instance + ": " + why);
            continue;
        }
        // Each body comes back where its own placement puts it; new bodies follow the first.
        const Placement first = instancePlacement(scene.bodies()[idx.front()]);
        std::map<std::string, Body> was;
        for (std::size_t i : idx) {
            const Body& b = scene.bodies()[i];
            if (b.mesh.vertices.size() != b.link.vertices) report.push_back(b.name + ": its shape changed here; the file's replaces it");
            was.emplace(b.link.body, b);
        }
        for (std::size_t k = idx.size(); k-- > 0;) scene.remove(idx[k]);
        const std::vector<std::string> made = addInstance(scene, part, file, instance, [&](const std::string& partBody) {
            const auto it = was.find(partBody);
            return it == was.end() ? first : instancePlacement(it->second);
        });
        // What the assembly gave each body (visibility, physics, what it hangs on) stays.
        for (const std::string& n : made)
            for (Body& b : scene.bodies()) {
                if (b.name != n) continue;
                const auto it = was.find(b.link.body);
                if (it == was.end()) break;
                b.visible = it->second.visible;
                b.physics = it->second.physics;
                if (!it->second.attachedTo.empty()) b.attachedTo = it->second.attachedTo;
                was.erase(it);
                break;
            }
        for (const auto& [partBody, old] : was) report.push_back(old.name + ": no longer in " + fs::path(file).filename().string() + ", removed");
        report.push_back("Updated " + instance + " from " + fs::path(file).filename().string());
        ++reloaded;
    }
    if (reloaded) scene.touchBodies();
    return reloaded;
}

} // namespace

std::uint64_t fileStamp(const std::string& path) {
    std::error_code ec;
    const auto size = fs::file_size(path, ec);
    if (ec) return 0;
    const auto time = fs::last_write_time(path, ec);
    if (ec) return 0;
    const auto t = static_cast<std::uint64_t>(time.time_since_epoch().count());
    return (t * 1000003u) ^ static_cast<std::uint64_t>(size);
}

bool insertComponent(Scene& scene, ProjectState& /*state*/, const std::string& file, std::string name, const ComponentPlace& at, std::vector<std::string>* made,
                     std::string* why, const std::string& assemblyFile) {
    std::error_code ec;
    if (!fs::exists(file, ec)) {
        if (why) *why = "No file " + file;
        return false;
    }
    const std::string path = canonical(file);
    if (!assemblyFile.empty() && sameFile(assemblyFile, path)) {
        if (why) *why = "An assembly can't contain itself";
        return false;
    }
    // Whether it contains this assembly, read from the files as they are (before bringing its
    // own parts up to date, which could change what it links).
    if (!assemblyFile.empty()) {
        Scene raw;
        ProjectState rawState;
        std::string err, warn;
        std::set<std::string> seen;
        if (loadProject(raw, rawState, path, &err, &warn) && reaches(raw, canonical(assemblyFile), seen)) {
            if (why) *why = fs::path(file).filename().string() + " contains this assembly (through its parts): it can't go into it";
            return false;
        }
    }
    PartFile part;
    std::set<std::string> visiting;
    if (!assemblyFile.empty()) visiting.insert(canonical(assemblyFile));
    if (!readPart(path, part, why, visiting)) return false;
    if (part.scene.bodies().empty()) {
        if (why) *why = fs::path(file).filename().string() + " has no bodies";
        return false;
    }
    std::set<std::string> seen;
    if (!assemblyFile.empty() && reaches(part.scene, canonical(assemblyFile), seen)) {
        if (why) *why = fs::path(file).filename().string() + " contains this assembly (through its parts): it can't go into it";
        return false;
    }
    if (name.empty()) {
        const std::string stem = fs::path(file).stem().string();
        for (int n = 1;; ++n)
            if (!nameTaken(scene, stem + "-" + std::to_string(n))) {
                name = stem + "-" + std::to_string(n);
                break;
            }
    } else if (nameTaken(scene, name)) {
        if (why) *why = "There is already a body or component named " + name;
        return false;
    }
    Placement m = eulerPlacement(decimal(at.rotation));
    m.t = decimal(at.at);
    const std::vector<std::string> names = addInstance(scene, part, path, name, [&](const std::string&) { return m; });
    if (made) *made = names;
    return true;
}

std::vector<ComponentInfo> listComponents(const Scene& scene) {
    std::vector<ComponentInfo> out;
    for (const Body& b : scene.bodies()) {
        if (!b.linked()) continue;
        auto it = std::find_if(out.begin(), out.end(), [&](const ComponentInfo& c) { return c.name == b.component; });
        if (it == out.end()) {
            ComponentInfo c;
            c.name = b.component, c.file = b.link.file;
            std::error_code ec;
            c.missing = !fs::exists(b.link.file, ec);
            out.push_back(c);
            it = out.end() - 1;
        }
        ++it->bodies;
        if (!it->missing && fileStamp(b.link.file) != b.link.stamp) it->changed = true;
    }
    return out;
}

int reloadComponents(Scene& scene, const std::string& name, bool force, std::vector<std::string>& report) {
    std::set<std::string> visiting;
    return reloadImpl(scene, name, force, report, visiting);
}

bool replaceComponent(Scene& scene, const std::string& name, const std::string& file, std::string* why) {
    const std::vector<std::size_t> idx = instanceBodies(scene, name);
    if (idx.empty()) {
        if (why) *why = "No linked component named " + name;
        return false;
    }
    PartFile part;
    std::set<std::string> visiting;
    const std::string path = canonical(file);
    if (!readPart(path, part, why, visiting)) return false;
    if (part.scene.bodies().empty()) {
        if (why) *why = fs::path(file).filename().string() + " has no bodies";
        return false;
    }
    // The new part's origin goes where the old one's is, turned the same way.
    const Placement m = instancePlacement(scene.bodies()[idx.front()]);
    const std::string instance = scene.bodies()[idx.front()].component;
    for (std::size_t k = idx.size(); k-- > 0;) scene.remove(idx[k]);
    addInstance(scene, part, path, instance, [&](const std::string&) { return m; });
    return true;
}

bool breakComponent(Scene& scene, const std::string& name, std::string* why) {
    const std::vector<std::size_t> idx = instanceBodies(scene, name);
    if (idx.empty()) {
        if (why) *why = "No linked component named " + name;
        return false;
    }
    for (std::size_t i : idx) scene.bodies()[i].link = Body::Link{};
    scene.touchBodies();
    return true;
}

bool moveComponent(Scene& scene, const std::string& name, const Vec3& by, std::string* why) {
    const std::vector<std::size_t> idx = instanceBodies(scene, name);
    if (idx.empty()) {
        if (why) *why = "No linked component named " + name;
        return false;
    }
    for (std::size_t i : idx) scene.bodies()[i].offset += by;
    return true;
}

bool rotateComponent(Scene& scene, const std::string& name, const Vec3& degrees, const Vec3* about, std::string* why) {
    const std::vector<std::size_t> idx = instanceBodies(scene, name);
    if (idx.empty()) {
        if (why) *why = "No linked component named " + name;
        return false;
    }
    const Vec3 pivot = about ? *about : scene.bodies()[idx.front()].offset;
    Placement turn = eulerPlacement(decimal(degrees));
    turn.t = decimal(pivot) - turn.point(decimal(pivot)); // about the pivot
    for (std::size_t i : idx) place(scene.bodies()[i], turn);
    scene.touchBodies();
    return true;
}

bool makeComponent(Scene& scene, const std::vector<std::string>& names, const std::string& name, const std::string& file, std::string* why) {
    if (names.empty()) {
        if (why) *why = "Name the bodies that make the part";
        return false;
    }
    PartFile part;
    std::vector<std::size_t> idx;
    for (const std::string& n : names) {
        std::size_t found = scene.bodies().size();
        for (std::size_t i = 0; i < scene.bodies().size(); ++i)
            if (lowerOf(scene.bodies()[i].name) == lowerOf(n)) found = i;
        if (found == scene.bodies().size()) {
            if (why) *why = "No body named " + n;
            return false;
        }
        if (scene.bodies()[found].linked()) {
            if (why) *why = n + " already comes from " + fs::path(scene.bodies()[found].link.file).filename().string();
            return false;
        }
        Body b = scene.bodies()[found];
        b.component.clear();
        b.attachedTo.clear();
        b.assembly.clear();
        part.scene.add(std::move(b));
        idx.push_back(found);
    }
    std::string err;
    if (!saveProject(part.scene, part.state, file, &err)) {
        if (why) *why = err;
        return false;
    }
    std::sort(idx.begin(), idx.end());
    idx.erase(std::unique(idx.begin(), idx.end()), idx.end());
    for (std::size_t k = idx.size(); k-- > 0;) scene.remove(idx[k]);
    // Read back (as a reload would), and linked where the bodies were: no move.
    PartFile saved;
    std::set<std::string> visiting;
    if (!readPart(canonical(file), saved, why, visiting)) return false;
    addInstance(scene, saved, canonical(file), name, [](const std::string&) { return Placement{}; });
    return true;
}

bool packProject(const Scene& scene, const ProjectState& state, const std::string& projectName, const std::string& folder, std::vector<std::string>* written,
                 std::string* why) {
    std::error_code ec;
    fs::create_directories(folder, ec);
    if (!fs::is_directory(folder, ec)) {
        if (why) *why = "Cannot make the folder " + folder;
        return false;
    }
    // Every file linked, at any depth: each gets a place in the folder (by its name; two
    // different files of one name get a number).
    std::map<std::string, std::string> dest; // original -> copy
    std::set<std::string> namesUsed{lowerOf(projectName)};
    std::vector<std::string> todo;
    auto want = [&](const Scene& s) {
        for (const Body& b : s.bodies())
            if (b.linked() && !dest.count(canonical(b.link.file))) {
                const std::string orig = canonical(b.link.file);
                std::string n = fs::path(orig).filename().string();
                for (int k = 2; namesUsed.count(lowerOf(n)); ++k) n = fs::path(orig).stem().string() + "-" + std::to_string(k) + ".sicdm";
                namesUsed.insert(lowerOf(n));
                dest[orig] = (fs::path(folder) / n).string();
                todo.push_back(orig);
            }
    };
    want(scene);
    auto relink = [&](Scene& s) {
        for (Body& b : s.bodies())
            if (b.linked()) {
                const auto it = dest.find(canonical(b.link.file));
                if (it != dest.end()) b.link.file = it->second;
            }
    };
    for (std::size_t k = 0; k < todo.size(); ++k) {
        Scene s;
        ProjectState st;
        std::string err, warn;
        if (!loadProject(s, st, todo[k], &err, &warn)) {
            if (why) *why = todo[k] + ": " + err;
            return false;
        }
        want(s);
        relink(s);
        if (!saveProject(s, st, dest[todo[k]], &err)) {
            if (why) *why = err;
            return false;
        }
        if (written) written->push_back(dest[todo[k]]);
    }
    Scene top = scene;
    relink(top);
    const std::string mainFile = (fs::path(folder) / projectName).string();
    std::string err;
    if (!saveProject(top, state, mainFile, &err)) {
        if (why) *why = err;
        return false;
    }
    if (written) written->insert(written->begin(), mainFile);
    return true;
}

} // namespace sicdm

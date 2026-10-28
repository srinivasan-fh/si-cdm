// COMPONENT: assemblies made of part files (Components.hpp).

#include "core/ComponentCommand.hpp"

#include "core/Command.hpp"
#include "core/Components.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"

#include <cctype>
#include <filesystem>

namespace sicdm {

namespace fs = std::filesystem;

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

// A file named relative to the open project's folder (or the working folder when unsaved).
std::string resolve(const Document& doc, std::string file) {
    if (fileKindOf(file) != FileKind::Project) file += ".sicdm";
    fs::path p(file);
    if (p.is_relative() && !doc.path.empty()) p = fs::path(doc.path).parent_path() / p;
    return fs::absolute(p).lexically_normal().string();
}

std::string vecText(const Vec3& v) {
    char b[96];
    std::snprintf(b, sizeof b, "(%g, %g, %g)", static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z));
    return b;
}

} // namespace

CommandOutput runComponentCommand(Document& doc, const std::vector<std::string>& args, const std::vector<bool>& quoted, ViewHost* view) {
    CommandOutput out;
    const std::string usage =
        "COMPONENT INSERT file [AS name] [AT x,y,z] [ROTATE rx,ry,rz] | LIST | RELOAD [name] [FORCE] | OPEN name | REPLACE name file | "
        "MAKE name FILE file FROM body... | BREAK name | MOVE name dx,dy,dz | ROTATE name rx,ry,rz [ABOUT x,y,z] | PACK folder";
    if (args.empty()) return out.fail(usage);
    const std::string sub = lowerOf(args[0]);
    auto word = [&](std::size_t i, const char* w) { return i < args.size() && !(i < quoted.size() && quoted[i]) && lowerOf(args[i]) == w; };
    std::string why;

    if (sub == "insert" || sub == "add") {
        if (args.size() < 2) return out.fail("COMPONENT INSERT file [AS name] [AT x,y,z] [ROTATE rx,ry,rz]");
        const std::string file = resolve(doc, args[1]);
        std::string name;
        ComponentPlace place;
        for (std::size_t i = 2; i < args.size(); ++i) {
            if (word(i, "as") && i + 1 < args.size()) {
                name = args[++i];
            } else if (word(i, "at") && i + 1 < args.size()) {
                const auto v = parseVec3(args[++i]);
                if (!v) return out.fail("AT needs x,y,z");
                place.at = *v;
            } else if (word(i, "rotate") && i + 1 < args.size()) {
                const auto v = parseVec3(args[++i]);
                if (!v) return out.fail("ROTATE needs rx,ry,rz (degrees)");
                place.rotation = *v;
            } else if (args[i].front() == '@') {
                const auto v = parseVec3(args[i].substr(1));
                if (!v) return out.fail("@ needs x,y,z");
                place.at = *v;
            } else {
                return out.fail("COMPONENT INSERT file [AS name] [AT x,y,z] [ROTATE rx,ry,rz]");
            }
        }
        std::vector<std::string> made;
        if (!insertComponent(doc.scene, doc.state, file, name, place, &made, &why, doc.path)) return out.fail(why);
        doc.dirty = true;
        doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
        const Body& b = doc.scene.bodies().back();
        out.say("Inserted " + b.component + " from " + fs::path(file).filename().string() + " at " + vecText(place.at) +
                (made.size() == 1 ? std::string() : " (" + std::to_string(made.size()) + " bodies)") + ", linked: it follows the file");
        return out;
    }
    if (sub == "list") {
        const auto list = listComponents(doc.scene);
        if (list.empty()) out.say("No linked components (COMPONENT INSERT part.sicdm adds one)");
        for (const ComponentInfo& c : list)
            out.say("  " + c.name + "  " + c.file + (c.bodies > 1 ? "  " + std::to_string(c.bodies) + " bodies" : std::string()) +
                    (c.missing ? "  [missing]" : c.changed ? "  [changed: COMPONENT RELOAD]" : "  [up to date]"));
        return out;
    }
    if (sub == "reload" || sub == "update") {
        const bool force = word(args.size() - 1, "force");
        const std::string name = args.size() > 1 && !word(1, "force") && !word(1, "all") ? args[1] : std::string();
        std::vector<std::string> report;
        const int n = reloadComponents(doc.scene, name, force, report);
        for (const std::string& l : report) out.say(l);
        if (n == 0 && report.empty()) out.say("Every component is up to date");
        if (n) doc.dirty = true;
        return out;
    }
    if (sub == "open" || sub == "edit") {
        if (args.size() < 2) return out.fail("COMPONENT OPEN name");
        for (const ComponentInfo& c : listComponents(doc.scene))
            if (lowerOf(c.name) == lowerOf(args[1])) {
                if (c.missing) return out.fail(c.file + " is missing");
                // In a tab of its own when there are tabs (the assembly stays open, and reloads it
                // when it's saved); headless, it replaces the project.
                return executeCommand(doc, "OPEN \"" + c.file + "\"", view);
            }
        return out.fail("No linked component named " + args[1]);
    }
    if (sub == "replace") {
        if (args.size() < 3) return out.fail("COMPONENT REPLACE name file");
        const std::string file = resolve(doc, args[2]);
        if (!replaceComponent(doc.scene, args[1], file, &why)) return out.fail(why);
        doc.dirty = true;
        out.say(args[1] + " now comes from " + fs::path(file).filename().string());
        return out;
    }
    if (sub == "make" || sub == "save") {
        // MAKE name FILE path FROM body...
        if (args.size() < 2) return out.fail("COMPONENT MAKE name FILE part.sicdm FROM body...");
        const std::string name = args[1];
        std::string file;
        std::vector<std::string> bodies;
        for (std::size_t i = 2; i < args.size(); ++i) {
            if (word(i, "file") && i + 1 < args.size()) file = resolve(doc, args[++i]);
            else if (word(i, "from")) {
                for (++i; i < args.size(); ++i) bodies.push_back(args[i]);
            } else return out.fail("COMPONENT MAKE name FILE part.sicdm FROM body...");
        }
        if (bodies.empty() && doc.selectedBody >= 0 && doc.selectedBody < static_cast<int>(doc.scene.bodies().size()))
            bodies.push_back(doc.scene.bodies()[static_cast<std::size_t>(doc.selectedBody)].name);
        if (file.empty()) file = resolve(doc, name);
        std::error_code ec;
        if (fs::exists(file, ec)) return out.fail(file + " already exists: choose another FILE (COMPONENT INSERT links an existing part)");
        if (!makeComponent(doc.scene, bodies, name, file, &why)) return out.fail(why);
        doc.dirty = true;
        doc.clearSelection();
        out.say("Saved " + std::to_string(bodies.size()) + (bodies.size() == 1 ? " body" : " bodies") + " as " + file + ", linked here as " + name);
        return out;
    }
    if (sub == "break" || sub == "unlink") {
        if (args.size() < 2) return out.fail("COMPONENT BREAK name");
        if (!breakComponent(doc.scene, args[1], &why)) return out.fail(why);
        doc.dirty = true;
        out.say(args[1] + " is no longer linked: its bodies are this project's own");
        return out;
    }
    if (sub == "move") {
        if (args.size() < 3) return out.fail("COMPONENT MOVE name dx,dy,dz");
        const auto d = parseVec3(args[2]);
        if (!d) return out.fail("COMPONENT MOVE name dx,dy,dz");
        if (!moveComponent(doc.scene, args[1], *d, &why)) return out.fail(why);
        doc.dirty = true;
        out.say("Moved " + args[1] + " by " + vecText(*d));
        return out;
    }
    if (sub == "rotate" || sub == "turn") {
        if (args.size() < 3) return out.fail("COMPONENT ROTATE name rx,ry,rz [ABOUT x,y,z]");
        const auto r = parseVec3(args[2]);
        if (!r) return out.fail("COMPONENT ROTATE name rx,ry,rz [ABOUT x,y,z]");
        std::optional<Vec3> about;
        if (word(3, "about") && args.size() > 4) {
            about = parseVec3(args[4]);
            if (!about) return out.fail("ABOUT needs x,y,z");
        }
        if (!rotateComponent(doc.scene, args[1], *r, about ? &*about : nullptr, &why)) return out.fail(why);
        doc.dirty = true;
        out.say("Turned " + args[1] + " by " + vecText(*r) + " deg");
        return out;
    }
    if (sub == "pack") {
        if (args.size() < 2) return out.fail("COMPONENT PACK folder");
        fs::path folder(args[1]);
        if (folder.is_relative() && !doc.path.empty()) folder = fs::path(doc.path).parent_path() / folder;
        const std::string name = doc.path.empty() ? std::string("assembly.sicdm") : fs::path(doc.path).filename().string();
        std::vector<std::string> written;
        if (!packProject(doc.scene, doc.state, name, folder.string(), &written, &why)) return out.fail(why);
        out.say("Packed " + std::to_string(written.size()) + (written.size() == 1 ? " file" : " files") + " into " + folder.string() + ":");
        for (const std::string& f : written) out.say("  " + fs::path(f).filename().string());
        return out;
    }
    return out.fail(usage);
}

} // namespace sicdm

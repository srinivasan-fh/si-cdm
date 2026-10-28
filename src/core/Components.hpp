#pragma once

// Assemblies made of part files. A part (or a sub-assembly) saved as its own project file is
// inserted into an assembly as a linked component: its bodies are copied in, grouped under
// the instance's name (Body::component; the mate solver moves them as one), and each body
// remembers the file, its name there and its pose there (Body::Link). When the file changes
// - edited in another tab, by a colleague, checked out of the vault - a reload brings in the
// new shape where the instance is now: the move, turn or mate it has in the assembly is kept
// (the body's pose here relative to its pose in the file, applied to the new one). Parts are
// reused: the same file may be inserted any number of times.
//
// Files are kept relative to the assembly when it is saved, so a folder of parts and their
// assembly can move together (COMPONENT PACK copies them into one).

#include "core/Math.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace sicdm {

class Scene;
struct ProjectState;

// A file's size and modification time, folded together: a cheap "has it changed?".
std::uint64_t fileStamp(const std::string& path);

struct ComponentPlace {
    Vec3 at{};       // where the part file's origin goes (mm)
    Vec3 rotation{}; // turned first, degrees about X, then Y, then Z, about that origin
};

// Inserts a part file as instance `name` ("" : the file's name and a number). The names of
// the bodies made (for one body, the instance's name; for several, "instance:body").
// False (why) when the file can't be read, is this project, or holds nothing.
bool insertComponent(Scene& scene, ProjectState& state, const std::string& file, std::string name, const ComponentPlace& place,
                     std::vector<std::string>* made = nullptr, std::string* why = nullptr, const std::string& assemblyFile = {});

struct ComponentInfo {
    std::string name, file;
    int bodies = 0;
    bool missing = false; // the file isn't there (the bodies keep their last shape)
    bool changed = false; // the file changed since it was loaded
};
std::vector<ComponentInfo> listComponents(const Scene& scene);

// Brings instances up to date with their files: `name` ("" every instance), only those whose
// file changed unless `force`. One line a reload or a problem in `report`; the count reloaded.
int reloadComponents(Scene& scene, const std::string& name, bool force, std::vector<std::string>& report);

// Swaps an instance's file for another, keeping where it is.
bool replaceComponent(Scene& scene, const std::string& name, const std::string& file, std::string* why = nullptr);

// Makes an instance's bodies the project's own (no longer linked).
bool breakComponent(Scene& scene, const std::string& name, std::string* why = nullptr);

// Moves (adds `by`) and turns (degrees, about `about`) an instance as one.
bool moveComponent(Scene& scene, const std::string& name, const Vec3& by, std::string* why = nullptr);
bool rotateComponent(Scene& scene, const std::string& name, const Vec3& degrees, const Vec3* about, std::string* why = nullptr);

// Saves these bodies as a new part file and links them as instance `name`, where they are.
bool makeComponent(Scene& scene, const std::vector<std::string>& bodies, const std::string& name, const std::string& file, std::string* why = nullptr);

// Pack and go: the project and every part file it links (and theirs), copied into `folder`
// with the links pointing at the copies. The files written in `written`.
bool packProject(const Scene& scene, const ProjectState& state, const std::string& projectName, const std::string& folder,
                 std::vector<std::string>* written = nullptr, std::string* why = nullptr);

} // namespace sicdm

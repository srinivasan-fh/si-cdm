#include "TestFramework.hpp"
#include "core/Assembly.hpp"
#include "core/Command.hpp"
#include "core/Components.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Scene.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <thread>

using namespace sicdm;
namespace fs = std::filesystem;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::string text(const CommandOutput& o) {
    std::string s;
    for (const std::string& l : o.lines) s += l + "\n";
    return s;
}

fs::path freshDir(const char* name) {
    const fs::path p = fs::temp_directory_path() / name;
    std::error_code ec;
    fs::remove_all(p, ec);
    fs::create_directories(p, ec);
    return p;
}

std::string q(const fs::path& p) { return "\"" + p.string() + "\""; }

const Body* named(const Document& d, const std::string& n) {
    for (const Body& b : d.scene.bodies())
        if (b.name == n) return &b;
    return nullptr;
}

Vec3 size(const Body& b) {
    const AABB a = b.bounds();
    return a.max - a.min;
}

Vec3 centre(const Body& b) {
    const AABB a = b.bounds();
    return (a.max + a.min) * 0.5f;
}

bool near(const Vec3& a, const Vec3& b, float tol = 1e-3f) { return std::fabs(a.x - b.x) < tol && std::fabs(a.y - b.y) < tol && std::fabs(a.z - b.z) < tol; }

// A later save of the part gets a different stamp even on file systems that keep whole seconds.
void touchLater(const fs::path& f) {
    std::error_code ec;
    fs::last_write_time(f, fs::last_write_time(f, ec) + std::chrono::seconds(2), ec);
}

} // namespace

TEST_CASE("components: a part file inserted twice, moved, turned, updated when the file changes") {
    const fs::path dir = freshDir("sicdm_components");
    // The part: a 40 x 20 x 10 block with its own part number.
    Document part;
    REQUIRE(run(part, "BOX 40 20 10 @0,0,0").ok);
    REQUIRE(run(part, "PROPERTY NUMBER P-10").ok);
    REQUIRE(run(part, "PROPERTY DESCRIPTION Spacer").ok);
    REQUIRE(run(part, "SAVE " + q(dir / "spacer.sicdm")).ok);

    Document asm1;
    REQUIRE(run(asm1, "COMPONENT INSERT " + q(dir / "spacer") + " AT 100,0,0").ok);
    REQUIRE(run(asm1, "COMPONENT INSERT " + q(dir / "spacer.sicdm") + " AS Right AT 200,0,0 ROTATE 0,0,90").ok);
    const Body* a = named(asm1, "spacer-1");
    const Body* b = named(asm1, "Right");
    REQUIRE(a && b);
    CHECK(a->linked());
    CHECK(near(size(*a), {40, 20, 10}));
    CHECK(near(size(*b), {20, 40, 10})); // turned a quarter
    CHECK(std::fabs(centre(*a).x - 100) < 1e-3f);
    CHECK(std::fabs(centre(*b).x - 200) < 1e-3f);
    // The parts list: one part, two of it, with the file's number.
    const auto bom = partsList(asm1.scene);
    REQUIRE(bom.size() == 1);
    CHECK(bom[0].quantity == 2);
    CHECK(bom[0].number == "P-10");
    CHECK(bom[0].description == "Spacer");
    // Placed here: moved and turned.
    REQUIRE(run(asm1, "MOVE 0,50,0 \"spacer-1\"").ok);
    REQUIRE(run(asm1, "COMPONENT ROTATE spacer-1 0,0,90").ok);
    CHECK(near(size(*named(asm1, "spacer-1")), {20, 40, 10}));
    const Vec3 placedAt = centre(*named(asm1, "spacer-1"));
    CHECK(text(run(asm1, "COMPONENT LIST")).find("[up to date]") != std::string::npos);
    // Saved beside the part, then the part changes: twice as long.
    REQUIRE(run(asm1, "SAVE " + q(dir / "frame.sicdm")).ok);
    REQUIRE(run(part, "SCALE \"Block 1\" 2").ok);
    REQUIRE(run(part, "SAVE").ok);
    touchLater(dir / "spacer.sicdm");
    CHECK(text(run(asm1, "COMPONENT LIST")).find("[changed") != std::string::npos);
    const CommandOutput r = run(asm1, "COMPONENT RELOAD");
    REQUIRE(r.ok);
    CHECK(text(r).find("Updated spacer-1") != std::string::npos);
    CHECK(text(r).find("Updated Right") != std::string::npos);
    // The new shape, where the instance was put here (its turn kept).
    CHECK(near(size(*named(asm1, "spacer-1")), {40, 80, 20}));
    CHECK(near(size(*named(asm1, "Right")), {40, 80, 20}));
    const Vec3 nowAt = centre(*named(asm1, "spacer-1"));
    CHECK(std::fabs(nowAt.z - 10) < 1e-3f); // on the floor, the scaled block twice as high
    CHECK(std::fabs(nowAt.x - placedAt.x) < 1e-3f);
    CHECK(text(run(asm1, "COMPONENT RELOAD")).find("up to date") != std::string::npos);
    // UNDO takes the reload back.
    REQUIRE(run(asm1, "UNDO").ok);
    CHECK(near(size(*named(asm1, "spacer-1")), {20, 40, 10}));
    REQUIRE(run(asm1, "REDO").ok);
    CHECK(near(size(*named(asm1, "spacer-1")), {40, 80, 20}));

    // Saved and opened again: the links are relative to the assembly, and a change made while
    // it was closed comes in when it opens.
    REQUIRE(run(asm1, "SAVE").ok);
    REQUIRE(run(part, "SCALE \"Block 1\" 0.5").ok);
    REQUIRE(run(part, "SAVE").ok);
    touchLater(dir / "spacer.sicdm");
    Document asm2;
    const CommandOutput opened = run(asm2, "OPEN " + q(dir / "frame.sicdm"));
    REQUIRE(opened.ok);
    CHECK(text(opened).find("Updated spacer-1") != std::string::npos);
    CHECK(near(size(*named(asm2, "spacer-1")), {20, 40, 10}));
    CHECK(std::fabs(centre(*named(asm2, "spacer-1")).x - placedAt.x) < 1e-3f);

    // Pack and go: the assembly and its part in one folder; the copy opens from there.
    const fs::path packed = dir / "packed";
    REQUIRE(run(asm2, "COMPONENT PACK " + q(packed)).ok);
    CHECK(fs::exists(packed / "frame.sicdm"));
    CHECK(fs::exists(packed / "spacer.sicdm"));
    Document asm3;
    REQUIRE(run(asm3, "OPEN " + q(packed / "frame.sicdm")).ok);
    REQUIRE(named(asm3, "spacer-1") != nullptr);
    std::error_code ec;
    CHECK(fs::equivalent(fs::path(named(asm3, "spacer-1")->link.file).parent_path(), packed, ec)); // (the folder however it is spelled)

    // A missing file: listed as such, and the bodies keep their last shape.
    fs::remove(packed / "spacer.sicdm");
    CHECK(text(run(asm3, "COMPONENT LIST")).find("[missing]") != std::string::npos);
    CHECK(text(run(asm3, "COMPONENT RELOAD")).find("missing") != std::string::npos);
    CHECK(named(asm3, "spacer-1") != nullptr);
}

TEST_CASE("components: make a part file from bodies, replace, break, multi-body parts and no cycles") {
    const fs::path dir = freshDir("sicdm_components2");
    Document d;
    REQUIRE(run(d, "SAVE " + q(dir / "machine.sicdm")).ok);
    REQUIRE(run(d, "CYLINDER 5 60 @10,20,0").ok);
    const Vec3 was = centre(d.scene.bodies().back());
    // Top-down: the shaft becomes a part file of its own, linked where it was.
    REQUIRE(run(d, "COMPONENT MAKE Shaft FILE shaft FROM \"Cylinder 1\"").ok);
    CHECK(fs::exists(dir / "shaft.sicdm"));
    REQUIRE(named(d, "Shaft") != nullptr);
    CHECK(named(d, "Shaft")->linked());
    CHECK(near(centre(*named(d, "Shaft")), was));
    CHECK(!run(d, "COMPONENT MAKE Other FILE shaft FROM \"Shaft\"").ok); // the file is there already
    // A two-body part: names are instance:body, and they move as one.
    Document two;
    REQUIRE(run(two, "BOX 10 10 10 @0,0,0").ok);
    REQUIRE(run(two, "BOX 10 10 10 @30,0,0").ok);
    REQUIRE(run(two, "SAVE " + q(dir / "pair.sicdm")).ok);
    REQUIRE(run(d, "COMPONENT INSERT pair AT 0,100,0").ok);
    REQUIRE(named(d, "pair-1:Block 1") != nullptr);
    REQUIRE(named(d, "pair-1:Block 2") != nullptr);
    REQUIRE(run(d, "MOVE 5,0,0 \"pair-1:Block 1\"").ok);
    CHECK(std::fabs(centre(*named(d, "pair-1:Block 2")).x - 35) < 1e-3f); // the other body came too
    CHECK(partsList(d.scene).size() == 2); // the shaft and the pair (one part, two bodies)
    // Replace: the pair's file for the shaft's, in the same place.
    REQUIRE(run(d, "COMPONENT REPLACE pair-1 shaft").ok);
    CHECK(named(d, "pair-1") != nullptr);
    CHECK(named(d, "pair-1:Block 1") == nullptr);
    // Break: the project's own from now on.
    REQUIRE(run(d, "COMPONENT BREAK pair-1").ok);
    CHECK(!named(d, "pair-1")->linked());
    CHECK(!run(d, "COMPONENT BREAK pair-1").ok);
    // An assembly can't contain itself, directly or through a part that links back.
    REQUIRE(run(d, "SAVE").ok);
    CHECK(!run(d, "COMPONENT INSERT machine").ok);
    Document loop;
    REQUIRE(run(loop, "COMPONENT INSERT " + q(dir / "machine.sicdm")).ok);
    REQUIRE(run(loop, "SAVE " + q(dir / "loop.sicdm")).ok);
    const CommandOutput looped = run(d, "COMPONENT INSERT loop");
    CHECK(!looped.ok);
    if (looped.ok) {
        std::printf("  inserted %s into %s:\n%s", (dir / "loop.sicdm").string().c_str(), d.path.c_str(), text(looped).c_str());
        Document lp;
        run(lp, "OPEN " + q(dir / "loop.sicdm"));
        for (const Body& b : lp.scene.bodies()) {
            std::error_code ec;
            std::printf("  loop body %s links %s (the assembly's file: %d)\n", b.name.c_str(), b.link.file.c_str(),
                        b.link.file.empty() ? -1 : static_cast<int>(fs::equivalent(b.link.file, d.path, ec)));
        }
    }
    // Released assemblies: listing is fine, inserting isn't.
    REQUIRE(run(d, "REVISION RELEASE").ok);
    CHECK(run(d, "COMPONENT LIST").ok);
    CHECK(!run(d, "COMPONENT INSERT pair").ok);
}

TEST_CASE("components: NEW starts a project without the last one's product data, variables or render set-up") {
    Document d;
    REQUIRE(run(d, "PROPERTY NUMBER P-1").ok);
    REQUIRE(run(d, "VAR w = 12").ok);
    REQUIRE(run(d, "CONFIG SAVE Big").ok);
    REQUIRE(run(d, "RENDER ENV sky").ok);
    REQUIRE(run(d, "REVISION NEXT").ok);
    REQUIRE(run(d, "NEW").ok);
    CHECK(d.state.product.properties.empty());
    CHECK(d.state.product.revision == "A");
    CHECK(d.state.variables.empty());
    CHECK(d.state.configs.empty());
    CHECK(d.state.render.environment == "studio");
}

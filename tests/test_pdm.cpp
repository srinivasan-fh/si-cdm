#include "TestFramework.hpp"
#include "core/Assembly.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Drawing.hpp"
#include "core/FileIO.hpp"
#include "core/Pdm.hpp"
#include "core/Preferences.hpp"
#include "core/Scene.hpp"

#include <cstdlib>
#include <filesystem>
#include <string>

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

void setEnv(const char* name, const std::string& value) {
#if defined(_WIN32)
    _putenv_s(name, value.c_str());
#else
    setenv(name, value.c_str(), 1);
#endif
}

fs::path freshDir(const char* name) {
    const fs::path p = fs::temp_directory_path() / name;
    std::error_code ec;
    fs::remove_all(p, ec);
    fs::create_directories(p, ec);
    return p;
}

std::string q(const fs::path& p) { return "\"" + p.string() + "\""; }

} // namespace

TEST_CASE("pdm: the next revision") {
    CHECK(nextRevision("A") == "B");
    CHECK(nextRevision("H") == "J"); // I, O, Q, S, X and Z are skipped (they read as digits)
    CHECK(nextRevision("R") == "T");
    CHECK(nextRevision("Y") == "AA");
    CHECK(nextRevision("Z") == "AA");
    CHECK(nextRevision("A1") == "A2");
    CHECK(nextRevision("AZ") == "BA");
    CHECK(nextRevision("01") == "02");
    CHECK(nextRevision("9") == "10");
    CHECK(nextRevision("1.9") == "1.10");
    CHECK(nextRevision("") == "A");
    CHECK(vaultKey("P-100 bracket/v2") == "P-100_bracket_v2");
    CHECK(vaultKey("../x") == "_x");
}

TEST_CASE("pdm: properties on the project and the parts, saved, undone, in the parts list and title block") {
    Document d;
    REQUIRE(run(d, "BOX 20 20 20 @0,0,0").ok);
    REQUIRE(run(d, "BOX 10 10 10 @40,0,0").ok);
    REQUIRE(run(d, "PROPERTY NUMBER P-100").ok);
    REQUIRE(run(d, "PROPERTY DESCRIPTION Motor bracket, left").ok);
    CHECK(d.state.product.properties["NUMBER"] == "P-100");
    CHECK(d.state.product.properties["DESCRIPTION"] == "Motor bracket, left");
    REQUIRE(run(d, "PROPERTY \"Block 1\" NUMBER B-1").ok);
    REQUIRE(run(d, "PROPERTY \"Block 2\" NUMBER B-1").ok); // the same part twice
    REQUIRE(run(d, "PROPERTY \"Block 1\" DESCRIPTION Spacer").ok);
    CHECK(d.scene.bodies()[0].properties.at("NUMBER") == "B-1");
    CHECK(text(run(d, "PROPERTY \"Block 1\"")).find("DESCRIPTION = Spacer") != std::string::npos);
    // The parts list puts the two bodies with one part number together.
    const auto bom = partsList(d.scene);
    REQUIRE(bom.size() == 1);
    CHECK(bom[0].quantity == 2);
    CHECK(bom[0].number == "B-1");
    CHECK(text(run(d, "ASSEMBLY BOM")).find("B-1") != std::string::npos);
    // UNDO takes a property back.
    REQUIRE(run(d, "PROPERTY \"Block 1\" DESCRIPTION Washer").ok);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.bodies()[0].properties.at("DESCRIPTION") == "Spacer");
    REQUIRE(run(d, "PROPERTY VENDOR Acme").ok);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(!d.state.product.properties.count("VENDOR"));
    REQUIRE(run(d, "PROPERTY DELETE DESCRIPTION").ok);
    CHECK(!d.state.product.properties.count("DESCRIPTION"));
    CHECK(!run(d, "PROPERTY DELETE NOTHING").ok);
    // The title block takes the part number and the revision.
    const Drawing shown = drawingWithProduct(d.state.drawing, d.state.product);
    CHECK(shown.title.at("PART") == "P-100");
    CHECK(shown.title.at("REV") == "A");
    // Saved and read back.
    REQUIRE(run(d, "REVISION RELEASE NOTE first issue").ok);
    const fs::path dir = freshDir("sicdm_pdm_props");
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, (dir / "p.sicdm").string(), &err));
    Scene s;
    ProjectState st;
    REQUIRE(loadProject(s, st, (dir / "p.sicdm").string(), &err));
    CHECK(st.product.properties.at("NUMBER") == "P-100");
    CHECK(st.product.revision == "A");
    CHECK(st.product.state == ReleaseState::Released);
    REQUIRE(st.product.history.size() == 1);
    CHECK(st.product.history[0].note == "first issue");
    CHECK(s.bodies()[0].properties.at("DESCRIPTION") == "Spacer");
}

TEST_CASE("pdm: a released revision can't be changed; the next one can") {
    Document d;
    REQUIRE(run(d, "BOX 20 20 20 @0,0,0").ok);
    REQUIRE(run(d, "REVISION REVIEW").ok);
    CHECK(d.state.product.state == ReleaseState::InReview);
    REQUIRE(run(d, "REVISION RELEASE").ok);
    CHECK(d.state.product.locked());
    const std::uint64_t before = d.scene.revision();
    // Changes are refused...
    CHECK(!run(d, "BOX 5 5 5 @50,0,0").ok);
    CHECK(!run(d, "MOVE 10,0,0 \"Block 1\"").ok);
    CHECK(!run(d, "MATERIAL \"Block 1\" steel").ok);
    CHECK(!run(d, "PROPERTY NUMBER X").ok);
    CHECK(!run(d, "LINE 0,0 10,10").ok);
    CHECK(!run(d, "UNDO").ok);
    CHECK(!run(d, "VAR w = 10").ok);
    CHECK(!run(d, "REVISION STATE in work").ok); // released stays released
    CHECK(!run(d, "REVISION SET C").ok);
    CHECK(d.scene.bodies().size() == 1);
    CHECK(d.scene.revision() == before);
    // ...asking isn't, nor using it.
    CHECK(run(d, "MATERIAL \"Block 1\"").ok);
    CHECK(run(d, "HISTORY").ok);
    CHECK(run(d, "PROPERTY").ok);
    CHECK(run(d, "ASSEMBLY BOM").ok);
    CHECK(run(d, "MASSPROPS").ok);
    // The next revision is in work again.
    REQUIRE(run(d, "REVISION NEXT NOTE wider").ok);
    CHECK(d.state.product.revision == "B");
    CHECK(d.state.product.state == ReleaseState::InWork);
    CHECK(run(d, "BOX 5 5 5 @50,0,0").ok);
    CHECK(d.scene.bodies().size() == 2);
    CHECK(text(run(d, "REVISION HISTORY")).find("wider") != std::string::npos);
    // Only a released revision becomes obsolete.
    CHECK(!run(d, "REVISION STATE obsolete").ok);
}

TEST_CASE("pdm: compare two versions") {
    Document d;
    REQUIRE(run(d, "BOX 20 20 20 @0,0,0").ok);
    REQUIRE(run(d, "CYLINDER 5 30 @40,0,0").ok);
    const fs::path dir = freshDir("sicdm_pdm_compare");
    const fs::path a = dir / "a.sicdm";
    REQUIRE(run(d, "SAVE " + q(a)).ok);
    CHECK(text(run(d, "COMPARE " + q(a))).find("No differences") != std::string::npos);
    REQUIRE(run(d, "MOVE 5,0,0 \"Block 1\"").ok);
    REQUIRE(run(d, "ERASE \"Cylinder 2\"").ok);
    REQUIRE(run(d, "SPHERE 4 @0,40,0").ok);
    REQUIRE(run(d, "PROPERTY NUMBER P-7").ok);
    const std::string diff = text(run(d, "COMPARE " + q(a)));
    CHECK(diff.find("Block 1: moved by 5") != std::string::npos);
    CHECK(diff.find("Removed Cylinder 2") != std::string::npos);
    CHECK(diff.find("Added Sphere") != std::string::npos);
    CHECK(diff.find("NUMBER") != std::string::npos);
    CHECK(diff.find("History:") != std::string::npos);
    const fs::path b = dir / "b.sicdm";
    REQUIRE(run(d, "SAVE " + q(b)).ok);
    CHECK(text(run(d, "COMPARE " + q(a) + " " + q(b))).find("Removed Cylinder 2") != std::string::npos);
    CHECK(!run(d, "COMPARE " + q(dir / "none.sicdm")).ok);
}

TEST_CASE("pdm: the vault: check in, check out, two people, released revisions kept") {
    const fs::path dir = freshDir("sicdm_pdm_vault");
    const fs::path vault = dir / "vault";
    fs::create_directories(vault);
    setEnv("SICDM_USER", "ann");
    Document ann;
    REQUIRE(run(ann, "VAULT FOLDER " + q(vault)).ok);
    REQUIRE(run(ann, "BOX 20 20 20 @0,0,0").ok);
    REQUIRE(run(ann, "PROPERTY NUMBER P-200").ok);
    REQUIRE(run(ann, "PROPERTY DESCRIPTION Gear housing").ok);
    CHECK(!run(ann, "VAULT CHECKIN").ok); // not saved yet
    fs::create_directories(dir / "ann");
    REQUIRE(run(ann, "SAVE " + q(dir / "ann" / "housing.sicdm")).ok);
    REQUIRE(run(ann, "VAULT CHECKIN NOTE first model").ok);
    CHECK(ann.state.product.vaultItem == "P-200");
    auto item = vaultItem(vault.string(), "P-200");
    REQUIRE(item.has_value());
    REQUIRE(item->revisions.size() == 1);
    CHECK(item->revisions[0].author == "ann");
    CHECK(item->revisions[0].note == "first model");
    CHECK(item->description == "Gear housing");
    CHECK(item->lockedBy.empty());
    // Changing it again needs a checkout.
    REQUIRE(run(ann, "BOX 5 5 5 @40,0,0").ok);
    CHECK(!run(ann, "VAULT CHECKIN").ok);

    // Bob checks it out: Ann can't, until he checks it in.
    setEnv("SICDM_USER", "bob");
    Document bob;
    fs::create_directories(dir / "bob");
    const CommandOutput co = run(bob, "VAULT CHECKOUT P-200 TO " + q(dir / "bob" / "housing.sicdm"));
    REQUIRE(co.ok);
    CHECK(bob.scene.bodies().size() == 1);
    CHECK(bob.state.product.vaultItem == "P-200");
    CHECK(vaultItem(vault.string(), "P-200")->lockedBy == "bob");
    setEnv("SICDM_USER", "ann");
    const CommandOutput refused = run(ann, "VAULT CHECKOUT P-200 TO " + q(dir / "ann" / "again.sicdm"));
    CHECK(!refused.ok);
    CHECK(text(refused).find("checked out by bob") != std::string::npos);
    CHECK(!run(ann, "VAULT CHECKIN").ok);
    CHECK(!run(ann, "VAULT UNLOCK P-200").ok); // not without FORCE
    setEnv("SICDM_USER", "bob");
    REQUIRE(run(bob, "CYLINDER 4 10 @0,30,0").ok);
    REQUIRE(run(bob, "REVISION RELEASE").ok);
    REQUIRE(run(bob, "VAULT CHECKIN NOTE released").ok);
    item = vaultItem(vault.string(), "P-200");
    REQUIRE(item->revisions.size() == 1);
    CHECK(item->revisions[0].state == ReleaseState::Released);
    CHECK(item->lockedBy.empty());

    // A released revision is never written again: the next one goes beside it.
    REQUIRE(run(bob, "VAULT CHECKOUT P-200 TO " + q(dir / "bob" / "housing.sicdm")).ok);
    CHECK(!run(bob, "VAULT CHECKIN").ok); // still revision A, released
    REQUIRE(run(bob, "REVISION NEXT").ok);
    REQUIRE(run(bob, "SPHERE 3 @0,0,40").ok);
    REQUIRE(run(bob, "VAULT CHECKIN NOTE rev B").ok);
    item = vaultItem(vault.string(), "P-200");
    REQUIRE(item->revisions.size() == 2);
    CHECK(item->revisions[0].revision == "A");
    CHECK(item->revisions[1].revision == "B");
    CHECK(text(run(bob, "VAULT HISTORY P-200")).find("rev B") != std::string::npos);
    CHECK(text(run(bob, "VAULT LIST")).find("P-200") != std::string::npos);
    // What changed since A.
    const std::string diff = text(run(bob, "COMPARE REV A"));
    CHECK(diff.find("Added Sphere") != std::string::npos);
    // An older revision: a copy, not a checkout.
    setEnv("SICDM_USER", "ann");
    Document carol;
    REQUIRE(run(carol, "VAULT GET P-200 REV A TO " + q(dir / "ann" / "rev-a.sicdm")).ok);
    CHECK(carol.state.product.revision == "A");
    CHECK(carol.scene.bodies().size() == 2);
    CHECK(vaultItem(vault.string(), "P-200")->lockedBy.empty());
    // A checkout can be given up, and an administrator can take one away.
    REQUIRE(run(carol, "VAULT CHECKOUT P-200 TO " + q(dir / "ann" / "b.sicdm")).ok);
    CHECK(vaultItem(vault.string(), "P-200")->lockedBy == "ann");
    REQUIRE(run(carol, "VAULT UNDO P-200").ok);
    CHECK(vaultItem(vault.string(), "P-200")->lockedBy.empty());
    REQUIRE(vaultLock(vault.string(), "P-200"));
    setEnv("SICDM_USER", "admin");
    CHECK(!vaultUnlock(vault.string(), "P-200", false));
    CHECK(vaultUnlock(vault.string(), "P-200", true));
    setEnv("SICDM_USER", "");
    preferences().vault.clear();
}

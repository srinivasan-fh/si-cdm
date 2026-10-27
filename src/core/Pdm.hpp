#pragma once

// Product data: part numbers and other properties (on the project and on each body),
// revisions and their release states, comparing two versions of a project, and a vault: a
// shared folder (a network drive, a synced folder) a team checks projects in and out of.
//
// The vault keeps no shared index that two people could write at once: each item is a
// folder holding one file per revision (rev-A.sicdm) with its record beside it
// (rev-A.json), the item's description (item.json), and while someone has it checked out a
// lock file naming them (checkout.lock, made exclusively). A released revision's files are
// never written again.

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "core/Licence.hpp" // todayIso()

namespace sicdm {

class Scene;
struct ProjectState;
struct Drawing;

enum class ReleaseState { InWork, InReview, Released, Obsolete };
const char* releaseStateName(ReleaseState s); // "in work", "in review", "released", "obsolete"
std::optional<ReleaseState> parseReleaseState(const std::string& word);

struct RevisionRecord {
    std::string revision, date, author, note;
    ReleaseState state = ReleaseState::InWork;
};

struct ProductData {
    std::map<std::string, std::string> properties; // the project's: NUMBER, DESCRIPTION, MATERIAL...
    std::string revision = "A";
    ReleaseState state = ReleaseState::InWork;
    std::vector<RevisionRecord> history; // each change of revision or state, oldest first
    std::string vaultItem;               // the vault item it was checked out of ("": none)
    // A released or obsolete revision can't be changed (REVISION NEXT starts the next one).
    bool locked() const noexcept { return state == ReleaseState::Released || state == ReleaseState::Obsolete; }
};

// The revision after this one: A -> B, H -> J (no I, O, Q, S, X, Z), Y -> AA, 01 -> 02, 9 -> 10,
// 1.9 -> 1.10, "" -> A.
std::string nextRevision(const std::string& revision);
// Who is working: $SICDM_USER, else the login name, with the computer's name ("ann@cad-07").
std::string currentUser();

// What differs in the second project from the first, a line each (bodies added, removed,
// reshaped, moved, recoloured; parameters, materials and properties; the revision; the
// history; variables; mates; the drawing). Empty: the same.
std::vector<std::string> compareProjects(const Scene& a, const ProjectState& sa, const Scene& b, const ProjectState& sb);

// The drawing with its title block's empty PART, REV, MATERIAL, TITLE and COMPANY filled
// from the project's properties (NUMBER, MATERIAL, DESCRIPTION, COMPANY) and revision.
Drawing drawingWithProduct(const Drawing& d, const ProductData& p);

// The vault.
struct VaultRevision {
    std::string revision, date, author, note;
    ReleaseState state = ReleaseState::InWork;
    std::string file; // the project file of this revision
};
struct VaultItem {
    std::string key, description;
    std::vector<VaultRevision> revisions; // oldest first
    std::string lockedBy, lockedSince;    // "" when it isn't checked out
};

// An item's folder name: the key with only letters, digits, '-', '_' and '.'.
std::string vaultKey(const std::string& name);
std::vector<VaultItem> vaultItems(const std::string& vault, std::string* why = nullptr);
std::optional<VaultItem> vaultItem(const std::string& vault, const std::string& key, std::string* why = nullptr);
// Puts a saved project file into the vault as its revision (replacing a revision still in
// work; never a released one) and ends the user's checkout. False (why) when someone else
// has it checked out or the revision is released.
bool vaultCheckIn(const std::string& vault, const std::string& key, const std::string& projectFile, const ProductData& product, const std::string& note,
                  std::string* why = nullptr);
// Checks an item out to the user (a lock others see); false (why) when someone else has it.
bool vaultLock(const std::string& vault, const std::string& key, std::string* why = nullptr);
// Ends a checkout: the user's own, or anyone's with `force` (an administrator's fix).
bool vaultUnlock(const std::string& vault, const std::string& key, bool force, std::string* why = nullptr);

} // namespace sicdm

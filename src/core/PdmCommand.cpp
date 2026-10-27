// PROPERTY, REVISION, COMPARE and VAULT: product data, revisions and the team's vault.

#include "core/PdmCommand.hpp"

#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Pdm.hpp"
#include "core/Preferences.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>

namespace sicdm {

namespace fs = std::filesystem;

namespace {

std::string lowerOf(const std::string& s) {
    std::string o;
    for (char c : s) o += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return o;
}

std::string upperOf(const std::string& s) {
    std::string o;
    for (char c : s) o += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return o;
}

std::string joined(const std::vector<std::string>& a, std::size_t from, std::size_t to = std::string::npos) {
    std::string s;
    for (std::size_t i = from; i < a.size() && i < to; ++i) s += (s.empty() ? "" : " ") + a[i];
    return s;
}

int bodyIndex(const Document& doc, const std::string& name) {
    const auto& bs = doc.scene.bodies();
    for (std::size_t i = 0; i < bs.size(); ++i)
        if (lowerOf(bs[i].name) == lowerOf(name)) return static_cast<int>(i);
    return -1;
}

void record(ProductData& p, const std::string& note) {
    RevisionRecord r;
    r.revision = p.revision, r.state = p.state, r.date = todayIso(), r.author = currentUser(), r.note = note;
    p.history.push_back(std::move(r));
}

std::string describe(const ProductData& p) {
    return "Revision " + p.revision + ", " + releaseStateName(p.state) + (p.vaultItem.empty() ? std::string() : ", vault item " + p.vaultItem);
}

// NOTE text (to the end, or to the next of `stops`); removed from `a`.
std::string takeNote(std::vector<std::string>& a) {
    for (std::size_t i = 0; i < a.size(); ++i)
        if (lowerOf(a[i]) == "note") {
            const std::string n = joined(a, i + 1);
            a.erase(a.begin() + static_cast<std::ptrdiff_t>(i), a.end());
            return n;
        }
    return {};
}

// WORD value; removed from `a` ("" when it isn't there).
std::string takeOption(std::vector<std::string>& a, const char* word) {
    for (std::size_t i = 0; i + 1 < a.size(); ++i)
        if (lowerOf(a[i]) == word) {
            const std::string v = a[i + 1];
            a.erase(a.begin() + static_cast<std::ptrdiff_t>(i), a.begin() + static_cast<std::ptrdiff_t>(i) + 2);
            return v;
        }
    return {};
}

bool takeFlag(std::vector<std::string>& a, const char* word) {
    for (std::size_t i = 0; i < a.size(); ++i)
        if (lowerOf(a[i]) == word) {
            a.erase(a.begin() + static_cast<std::ptrdiff_t>(i));
            return true;
        }
    return false;
}

const VaultRevision* findRevision(const VaultItem& item, const std::string& rev) {
    if (rev.empty()) return item.revisions.empty() ? nullptr : &item.revisions.back();
    for (const VaultRevision& r : item.revisions)
        if (upperOf(r.revision) == upperOf(rev)) return &r;
    return nullptr;
}

std::string describeItem(const VaultItem& item) {
    std::string s = item.key;
    if (!item.revisions.empty()) {
        const VaultRevision& r = item.revisions.back();
        s += "  rev " + r.revision + " (" + releaseStateName(r.state) + ", " + r.date + ", " + r.author + ")";
    }
    if (!item.description.empty()) s += "  " + item.description;
    if (!item.lockedBy.empty()) s += "  [checked out by " + item.lockedBy + (item.lockedSince.empty() ? "" : " since " + item.lockedSince) + "]";
    return s;
}

// Copies a revision out of the vault to `to` (made read-write) and opens it.
CommandOutput fetchRevision(Document& doc, const VaultItem& item, const VaultRevision& r, std::string to, ViewHost* view, CommandOutput out) {
    if (to.empty()) {
        const fs::path base = doc.path.empty() ? fs::current_path() : fs::path(doc.path).parent_path();
        to = (base / (item.key + ".sicdm")).string();
    }
    if (fileKindOf(to) != FileKind::Project) to += ".sicdm";
    std::error_code ec;
    if (fs::absolute(to, ec) == fs::absolute(doc.path, ec) && doc.dirty)
        return out.fail(to + " is open with unsaved changes: SAVE or SAVEAS first, or VAULT CHECKOUT ... TO another file");
    fs::copy_file(r.file, to, fs::copy_options::overwrite_existing, ec);
    if (ec) return out.fail("Cannot copy " + r.file + " to " + to + ": " + ec.message());
    fs::permissions(to, fs::perms::owner_read | fs::perms::owner_write, fs::perm_options::add, ec);
    CommandOutput opened = executeCommand(doc, "OPEN \"" + to + "\"", view);
    for (std::string& l : opened.lines) out.say(std::move(l));
    out.ok = opened.ok;
    out.documentReplaced = opened.documentReplaced;
    return out;
}

} // namespace

std::string vaultFolder() {
    if (!preferences().vault.empty()) return preferences().vault;
    if (const char* v = std::getenv("SICDM_VAULT"); v && *v) return v;
    return {};
}

CommandOutput runPdmCommand(Document& doc, const std::string& cmd, const std::vector<std::string>& args, const std::vector<bool>& quoted, ViewHost* view) {
    CommandOutput out;
    std::vector<std::string> a = args;
    ProductData& pd = doc.state.product;

    if (cmd == "PROPERTY") {
        // PROPERTY [PROJECT | body] [LIST] | key value... | DELETE key
        std::map<std::string, std::string>* props = &pd.properties;
        std::string who = "Project";
        std::size_t i = 0;
        if (!a.empty() && lowerOf(a[0]) == "project" && !(!quoted.empty() && quoted[0])) {
            i = 1;
        } else if (!a.empty()) {
            const int bi = bodyIndex(doc, a[0]);
            if (bi >= 0) { // a body's (its name quoted when it has spaces)
                props = &doc.scene.bodies()[static_cast<std::size_t>(bi)].properties;
                who = doc.scene.bodies()[static_cast<std::size_t>(bi)].name;
                i = 1;
            }
        }
        if (i >= a.size() || lowerOf(a[i]) == "list") {
            if (props->empty()) out.say(who + ": no properties");
            else
                for (const auto& [k, v] : *props) out.say(who + ": " + k + " = " + v);
            if (props == &pd.properties) out.say(describe(pd));
            return out;
        }
        if (pd.locked())
            return out.fail(std::string("Revision ") + pd.revision + " is " + releaseStateName(pd.state) + ": its properties can't change (REVISION NEXT starts the next one)");
        if (lowerOf(a[i]) == "delete") {
            if (i + 1 >= a.size()) return out.fail("PROPERTY [body] DELETE key");
            const std::string key = upperOf(a[i + 1]);
            if (!props->erase(key)) return out.fail(who + " has no property " + key);
            doc.scene.touchBodies();
            doc.dirty = true;
            out.say(who + ": " + key + " removed");
            return out;
        }
        const std::string key = upperOf(a[i]);
        if (key.empty() || key.find_first_of(" =\"") != std::string::npos) return out.fail("A property's name is one word (NUMBER, DESCRIPTION, VENDOR...)");
        if (i + 1 >= a.size()) {
            const auto it = props->find(key);
            if (it == props->end()) return out.fail(who + " has no property " + key);
            out.say(who + ": " + key + " = " + it->second);
            return out;
        }
        const std::string value = joined(a, i + 1);
        (*props)[key] = value;
        doc.scene.touchBodies();
        doc.dirty = true;
        out.say(who + ": " + key + " = " + value);
        return out;
    }

    if (cmd == "REVISION") {
        // REVISION | STATE in-work|review|released|obsolete [NOTE text] | RELEASE [NOTE text] | NEXT [NOTE text] | SET label | HISTORY
        const std::string note = takeNote(a);
        const std::string sub = a.empty() ? "show" : lowerOf(a[0]);
        if (sub == "show" || sub == "info") {
            out.say(describe(pd));
            return out;
        }
        if (sub == "history" || sub == "log") {
            if (pd.history.empty()) out.say("No revision history yet: " + describe(pd));
            for (const RevisionRecord& r : pd.history)
                out.say("  " + r.revision + "  " + releaseStateName(r.state) + "  " + r.date + "  " + r.author + (r.note.empty() ? "" : "  " + r.note));
            return out;
        }
        if (sub == "next") {
            const std::string was = pd.revision;
            pd.revision = nextRevision(pd.revision);
            pd.state = ReleaseState::InWork;
            record(pd, note.empty() ? "from " + was : note);
            doc.dirty = true;
            out.say("Revision " + pd.revision + " started (in work), after " + was);
            return out;
        }
        if (sub == "set") {
            if (a.size() < 2) return out.fail("REVISION SET label (A, B, 01, 1.2...)");
            if (pd.locked()) return out.fail("Revision " + pd.revision + " is " + releaseStateName(pd.state) + ": REVISION NEXT starts the next one");
            const std::string label = upperOf(a[1]);
            if (label.find_first_of(" /\\\"") != std::string::npos) return out.fail("A revision is one word: letters, digits, '.' or '-'");
            pd.revision = label;
            record(pd, note.empty() ? "revision set" : note);
            doc.dirty = true;
            out.say(describe(pd));
            return out;
        }
        std::string word = sub == "state" ? (a.size() > 1 ? joined(a, 1) : std::string()) : joined(a, 0);
        const std::string lw = lowerOf(word);
        if (lw == "release") word = "released";
        else if (lw == "review" || lw == "submit") word = "in review";
        else if (lw == "reject" || lw == "rework" || lw == "work") word = "in work";
        else if (lw == "obsolete" || lw == "retire") word = "obsolete";
        const auto st = parseReleaseState(word);
        if (!st) return out.fail("REVISION | STATE IN-WORK|REVIEW|RELEASED|OBSOLETE [NOTE text] | RELEASE | NEXT | SET label | HISTORY");
        if (*st == pd.state) {
            out.say(describe(pd));
            return out;
        }
        if (pd.state == ReleaseState::Obsolete) return out.fail("Revision " + pd.revision + " is obsolete: REVISION NEXT starts a new one");
        if (pd.state == ReleaseState::Released && *st != ReleaseState::Obsolete)
            return out.fail("Revision " + pd.revision + " is released and stays as it is: REVISION NEXT starts revision " + nextRevision(pd.revision));
        if (*st == ReleaseState::Obsolete && pd.state != ReleaseState::Released) return out.fail("Only a released revision becomes obsolete");
        pd.state = *st;
        record(pd, note);
        doc.dirty = true;
        out.say(describe(pd) + (pd.locked() ? ": it can no longer be changed" : ""));
        return out;
    }

    if (cmd == "COMPARE") {
        // COMPARE file | COMPARE file1 file2 | COMPARE REV x [key]  (the open project against another version)
        Scene sa, sb;
        ProjectState pa, pb;
        const Scene* A = nullptr;
        const ProjectState* PA = nullptr;
        const Scene* B = &doc.scene;
        const ProjectState* PB = &doc.state;
        std::string nameA, nameB = doc.path.empty() ? std::string("the open project") : fs::path(doc.path).filename().string();
        std::string err, warn;
        auto load = [&](const std::string& f, Scene& s, ProjectState& p) {
            if (!loadProject(s, p, f, &err, &warn)) {
                out.fail(err);
                return false;
            }
            return true;
        };
        if (!a.empty() && lowerOf(a[0]) == "rev") {
            if (a.size() < 2) return out.fail("COMPARE REV x [vault item]");
            const std::string key = a.size() > 2 ? joined(a, 2) : pd.vaultItem;
            if (key.empty()) return out.fail("This project didn't come from the vault: COMPARE REV x item");
            std::string why;
            const auto item = vaultItem(vaultFolder(), key, &why);
            if (!item) return out.fail(why);
            const VaultRevision* r = findRevision(*item, a[1]);
            if (!r) return out.fail("No revision " + a[1] + " of " + key + " in the vault");
            if (!load(r->file, sa, pa)) return out;
            A = &sa, PA = &pa, nameA = key + " rev " + r->revision;
        } else if (a.size() == 2 && fileKindOf(a[0]) == FileKind::Project && fileKindOf(a[1]) == FileKind::Project) {
            if (!load(a[0], sa, pa) || !load(a[1], sb, pb)) return out;
            A = &sa, PA = &pa, B = &sb, PB = &pb, nameA = a[0], nameB = a[1];
        } else if (!a.empty()) {
            const std::string f = joined(a, 0);
            if (!load(f, sa, pa)) return out;
            A = &sa, PA = &pa, nameA = f;
        } else {
            return out.fail("COMPARE file.sicdm | COMPARE first.sicdm second.sicdm | COMPARE REV x [vault item]");
        }
        const auto diff = compareProjects(*A, *PA, *B, *PB);
        if (diff.empty()) {
            out.say("No differences between " + nameA + " and " + nameB);
            return out;
        }
        out.say(std::to_string(diff.size()) + (diff.size() == 1 ? " difference" : " differences") + " from " + nameA + " to " + nameB + ":");
        for (const std::string& l : diff) out.say("  " + l);
        return out;
    }

    if (cmd == "VAULT") {
        // VAULT [LIST] | FOLDER path | CHECKIN [AS key] [NOTE text] | CHECKOUT key [TO file] | GET key [REV x] [TO file]
        //       | UNDO [key] | UNLOCK key FORCE | HISTORY [key]
        const std::string note = takeNote(a);
        const std::string vault = vaultFolder();
        const std::string sub = a.empty() ? "list" : lowerOf(a[0]);
        std::string why;
        auto keyArg = [&](std::size_t from) { return from < a.size() ? joined(a, from) : pd.vaultItem; };
        if (sub == "folder") {
            if (a.size() < 2) {
                out.say(vault.empty() ? std::string("No vault folder (VAULT FOLDER path sets it)") : "Vault: " + vault);
                return out;
            }
            std::string err;
            if (!setPreference(preferences(), "vault", joined(a, 1), &err) || !savePreferences(&err)) return out.fail(err);
            std::error_code ec;
            const std::string f = preferences().vault;
            if (!f.empty() && !fs::is_directory(f, ec)) fs::create_directories(f, ec);
            out.say(f.empty() ? std::string("No vault folder") : "Vault: " + f + (fs::is_directory(f, ec) ? "" : " (not reachable yet)"));
            return out;
        }
        if (sub == "list") {
            const auto items = vaultItems(vault, &why);
            if (!why.empty()) return out.fail(why);
            out.say("Vault " + vault + ": " + std::to_string(items.size()) + (items.size() == 1 ? " item" : " items"));
            for (const VaultItem& it : items) out.say("  " + describeItem(it));
            return out;
        }
        if (sub == "history") {
            const std::string key = keyArg(1);
            if (key.empty()) return out.fail("VAULT HISTORY item");
            const auto item = vaultItem(vault, key, &why);
            if (!item) return out.fail(why);
            out.say(describeItem(*item));
            for (const VaultRevision& r : item->revisions)
                out.say("  " + r.revision + "  " + releaseStateName(r.state) + "  " + r.date + "  " + r.author + (r.note.empty() ? "" : "  " + r.note));
            return out;
        }
        if (sub == "checkin" || sub == "in") {
            a.erase(a.begin());
            std::string key = takeOption(a, "as");
            if (key.empty()) key = pd.vaultItem;
            if (key.empty()) {
                const auto n = pd.properties.find("NUMBER");
                key = n != pd.properties.end() ? n->second : doc.path.empty() ? std::string() : fs::path(doc.path).stem().string();
            }
            if (key.empty()) return out.fail("SAVE the project first, or give it a part NUMBER (PROPERTY NUMBER ...), or VAULT CHECKIN AS key");
            if (doc.path.empty()) return out.fail("SAVE the project to a file first: the vault keeps a copy of it");
            key = vaultKey(key);
            const std::string had = pd.vaultItem;
            pd.vaultItem = key; // saved with it, so a later checkout knows where it came from
            std::string err;
            if (!saveProject(doc.scene, doc.state, doc.path, &err)) {
                pd.vaultItem = had;
                return out.fail(err);
            }
            if (!vaultCheckIn(vault, key, doc.path, pd, note, &why)) {
                pd.vaultItem = had;
                saveProject(doc.scene, doc.state, doc.path, &err);
                return out.fail(why);
            }
            doc.dirty = false;
            out.say("Checked in " + key + " revision " + pd.revision + " (" + releaseStateName(pd.state) + ") to " + vault);
            return out;
        }
        if (sub == "checkout" || sub == "out" || sub == "get") {
            a.erase(a.begin());
            const std::string to = takeOption(a, "to"), rev = takeOption(a, "rev");
            const std::string key = joined(a, 0);
            if (key.empty()) return out.fail(sub == "get" ? "VAULT GET item [REV x] [TO file]" : "VAULT CHECKOUT item [TO file]");
            const auto item = vaultItem(vault, key, &why);
            if (!item) return out.fail(why);
            const VaultRevision* r = findRevision(*item, rev);
            if (!r) return out.fail(rev.empty() ? item->key + " has no revisions" : "No revision " + rev + " of " + item->key);
            if (sub != "get") {
                if (r != &item->revisions.back()) return out.fail("Only the latest revision is checked out; VAULT GET " + item->key + " REV " + rev + " gets a copy");
                if (!vaultLock(vault, item->key, &why)) return out.fail(why);
                out.say("Checked out " + item->key + " revision " + r->revision + " to " + currentUser());
                if (r->state == ReleaseState::Released) out.say("Revision " + r->revision + " is released: REVISION NEXT starts " + nextRevision(r->revision));
            } else {
                out.say("A copy of " + item->key + " revision " + r->revision + " (not checked out: VAULT CHECKIN won't take it)");
            }
            CommandOutput res = fetchRevision(doc, *item, *r, to, view, std::move(out));
            if (!res.ok && sub != "get") vaultUnlock(vault, item->key, false, nullptr);
            return res;
        }
        if (sub == "undo" || sub == "release-lock" || sub == "cancel") {
            const std::string key = keyArg(1);
            if (key.empty()) return out.fail("VAULT UNDO item");
            if (!vaultUnlock(vault, key, false, &why)) return out.fail(why);
            out.say(key + " is no longer checked out (the vault's copy is unchanged)");
            return out;
        }
        if (sub == "unlock") {
            const bool force = takeFlag(a, "force");
            const std::string key = keyArg(1);
            if (key.empty()) return out.fail("VAULT UNLOCK item FORCE");
            if (!vaultUnlock(vault, key, force, &why)) return out.fail(why);
            out.say(key + " unlocked");
            return out;
        }
        return out.fail("VAULT [LIST] | FOLDER path | CHECKIN [AS key] [NOTE text] | CHECKOUT key [TO file] | GET key [REV x] [TO file] | UNDO [key] | UNLOCK key FORCE | HISTORY [key]");
    }
    return out.fail("Unknown command " + cmd);
}

} // namespace sicdm

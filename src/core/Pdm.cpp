#include "core/Pdm.hpp"

#include "core/FileIO.hpp"
#include "core/Json.hpp"
#include "core/Licence.hpp"
#include "core/Scene.hpp"
#include "core/Sculpt.hpp"
#include "core/Translate.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>

#if !defined(_WIN32)
#include <unistd.h>
#endif

namespace sicdm {

namespace fs = std::filesystem;

const char* releaseStateName(ReleaseState s) {
    switch (s) {
    case ReleaseState::InWork: return SICDM_TR_NOOP("in work");
    case ReleaseState::InReview: return SICDM_TR_NOOP("in review");
    case ReleaseState::Released: return SICDM_TR_NOOP("released");
    case ReleaseState::Obsolete: return SICDM_TR_NOOP("obsolete");
    }
    return "?";
}

std::optional<ReleaseState> parseReleaseState(const std::string& word) {
    std::string w;
    for (char c : word)
        if (std::isalpha(static_cast<unsigned char>(c))) w += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (w == "inwork" || w == "work" || w == "wip" || w == "draft") return ReleaseState::InWork;
    if (w == "inreview" || w == "review") return ReleaseState::InReview;
    if (w == "released" || w == "release") return ReleaseState::Released;
    if (w == "obsolete") return ReleaseState::Obsolete;
    return std::nullopt;
}

std::string nextRevision(const std::string& r) {
    if (r.empty() || r == "-") return "A";
    // Trailing digits count up, keeping their width: 01 -> 02, 9 -> 10, 1.9 -> 1.10, A1 -> A2.
    std::size_t d = r.size();
    while (d > 0 && std::isdigit(static_cast<unsigned char>(r[d - 1]))) --d;
    if (d < r.size()) {
        const std::string digits = r.substr(d), n = std::to_string(std::stoull(digits.substr(0, 18)) + 1);
        return r.substr(0, d) + (n.size() < digits.size() ? std::string(digits.size() - n.size(), '0') + n : n);
    }
    // Letters count like a spreadsheet's columns (Y -> AA, AY -> BA), without I, O, Q, S, X
    // and Z, which read as digits (ASME Y14.35).
    static const std::string kLetters = "ABCDEFGHJKLMNPRTUVWY";
    std::string s = r;
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    if (!std::all_of(s.begin(), s.end(), [](char c) { return c >= 'A' && c <= 'Z'; })) return r + ".1"; // a scheme of its own
    for (std::size_t i = s.size(); i-- > 0;) {
        const auto next = std::find_if(kLetters.begin(), kLetters.end(), [&](char c) { return c > s[i]; });
        if (next != kLetters.end()) {
            s[i] = *next;
            return s;
        }
        s[i] = 'A'; // past the last letter: carry
    }
    return "A" + s;
}

std::string currentUser() {
    auto env = [](const char* k) -> std::string {
        const char* v = std::getenv(k);
        return v ? v : "";
    };
    if (const std::string u = env("SICDM_USER"); !u.empty()) return u;
    std::string user = env("USER");
    if (user.empty()) user = env("USERNAME");
    if (user.empty()) user = "someone";
    std::string host = env("COMPUTERNAME");
#if !defined(_WIN32)
    if (host.empty()) {
        char buf[256] = {};
        if (gethostname(buf, sizeof(buf) - 1) == 0) host = buf;
    }
#endif
    return host.empty() ? user : user + "@" + host;
}

namespace {

std::string num(double v) {
    char b[64];
    std::snprintf(b, sizeof(b), "%.4g", v);
    return b;
}

bool sameVec(const Vec3& a, const Vec3& b, float tol) { return std::fabs(a.x - b.x) <= tol && std::fabs(a.y - b.y) <= tol && std::fabs(a.z - b.z) <= tol; }

void compareMaps(const std::map<std::string, std::string>& a, const std::map<std::string, std::string>& b, const std::string& who, std::vector<std::string>& out) {
    for (const auto& [k, v] : b) {
        const auto it = a.find(k);
        if (it == a.end()) out.push_back(who + k + " set to \"" + v + "\"");
        else if (it->second != v) out.push_back(who + k + " \"" + it->second + "\" -> \"" + v + "\"");
    }
    for (const auto& [k, v] : a)
        if (!b.count(k)) out.push_back(who + k + " removed (was \"" + v + "\")");
}

} // namespace

std::vector<std::string> compareProjects(const Scene& a, const ProjectState& sa, const Scene& b, const ProjectState& sb) {
    std::vector<std::string> out;
    // The revision and the project's properties.
    if (sa.product.revision != sb.product.revision || sa.product.state != sb.product.state)
        out.push_back("Revision " + sa.product.revision + " (" + releaseStateName(sa.product.state) + ") -> " + sb.product.revision + " (" +
                      releaseStateName(sb.product.state) + ")");
    compareMaps(sa.product.properties, sb.product.properties, "Project ", out);
    // Bodies, by name.
    std::map<std::string, const Body*> before, after;
    for (const Body& x : a.bodies()) before[x.name] = &x;
    for (const Body& x : b.bodies()) after[x.name] = &x;
    for (const auto& [name, x] : before)
        if (!after.count(name)) out.push_back("Removed " + name);
    for (const auto& [name, y] : after) {
        const auto it = before.find(name);
        if (it == before.end()) {
            out.push_back("Added " + name + " (" + num(meshVolume(y->mesh)) + " mm3)");
            continue;
        }
        const Body& x = *it->second;
        if (x.kind != y->kind || x.params != y->params) out.push_back(name + ": parameters changed");
        if (meshContentKey(x.mesh) != meshContentKey(y->mesh)) {
            const double va = meshVolume(x.mesh), vb = meshVolume(y->mesh);
            const AABB ba = x.mesh.bounds(), bb = y->mesh.bounds();
            const Vec3 sa3 = ba.max - ba.min, sb3 = bb.max - bb.min;
            std::string line = name + ": shape changed";
            if (std::fabs(va - vb) > 1e-6 * std::max(std::fabs(va), 1.0)) line += ", volume " + num(va) + " -> " + num(vb) + " mm3";
            if (!sameVec(sa3, sb3, 1e-4f)) line += ", size " + num(sa3.x) + " x " + num(sa3.y) + " x " + num(sa3.z) + " -> " + num(sb3.x) + " x " + num(sb3.y) + " x " + num(sb3.z);
            out.push_back(line);
        }
        if (!sameVec(x.offset, y->offset, 1e-4f)) {
            const Vec3 d = y->offset - x.offset;
            out.push_back(name + ": moved by " + num(d.x) + ", " + num(d.y) + ", " + num(d.z));
        }
        if (!sameVec(x.rotation, y->rotation, 1e-4f)) out.push_back(name + ": turned");
        if (!sameVec(x.color, y->color, 1e-4f)) out.push_back(name + ": colour changed");
        if (x.material.preset != y->material.preset || x.material.roughness != y->material.roughness || x.material.metallic != y->material.metallic)
            out.push_back(name + ": material " + (x.material.preset.empty() ? std::string("plastic") : x.material.preset) + " -> " +
                          (y->material.preset.empty() ? std::string("plastic") : y->material.preset));
        if (x.visible != y->visible) out.push_back(name + (y->visible ? ": shown" : ": hidden"));
        compareMaps(x.properties, y->properties, name + ": ", out);
    }
    // Variables.
    std::map<std::string, std::string> va, vb;
    for (const Variable& v : sa.variables) va[v.name] = v.expr;
    for (const Variable& v : sb.variables) vb[v.name] = v.expr;
    compareMaps(va, vb, "Variable ", out);
    // The history: steps one has and the other hasn't (as many times as they occur).
    std::multiset<std::string> ha, hb;
    for (const HistoryStep& s : sa.history) ha.insert(s.line);
    for (const HistoryStep& s : sb.history) hb.insert(s.line);
    std::size_t added = 0, removed = 0;
    for (auto it = hb.begin(); it != hb.end(); it = hb.upper_bound(*it))
        if (ha.count(*it) < hb.count(*it)) added += hb.count(*it) - ha.count(*it);
    for (auto it = ha.begin(); it != ha.end(); it = ha.upper_bound(*it))
        if (hb.count(*it) < ha.count(*it)) removed += ha.count(*it) - hb.count(*it);
    if (added || removed) out.push_back("History: " + num(static_cast<double>(added)) + " steps added, " + num(static_cast<double>(removed)) + " taken out");
    if (sa.assembly.mates.size() != sb.assembly.mates.size())
        out.push_back("Mates: " + num(static_cast<double>(sa.assembly.mates.size())) + " -> " + num(static_cast<double>(sb.assembly.mates.size())));
    if (sa.drawing.views.size() != sb.drawing.views.size())
        out.push_back("Drawing views: " + num(static_cast<double>(sa.drawing.views.size())) + " -> " + num(static_cast<double>(sb.drawing.views.size())));
    return out;
}

Drawing drawingWithProduct(const Drawing& d, const ProductData& p) {
    Drawing out = d;
    auto fill = [&](const char* key, const std::string& v) {
        if (v.empty()) return;
        auto it = out.title.find(key);
        if (it == out.title.end() || it->second.empty()) out.title[key] = v;
    };
    auto prop = [&](const char* key) {
        const auto it = p.properties.find(key);
        return it == p.properties.end() ? std::string() : it->second;
    };
    fill("PART", prop("NUMBER"));
    fill("MATERIAL", prop("MATERIAL"));
    fill("TITLE", prop("DESCRIPTION"));
    fill("COMPANY", prop("COMPANY"));
    fill("REV", p.revision);
    return out;
}

// ---------------------------------------------------------------------------
// The vault

std::string vaultKey(const std::string& name) {
    std::string k;
    for (char c : name) {
        const unsigned char u = static_cast<unsigned char>(c);
        k += std::isalnum(u) || c == '-' || c == '_' || c == '.' ? c : '_';
    }
    while (!k.empty() && k.front() == '.') k.erase(k.begin()); // no hidden or ".." folders
    return k.empty() ? "item" : k;
}

namespace {

std::optional<Json> readJson(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    if (!in) return std::nullopt;
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return Json::parse(text);
}

bool readLock(const fs::path& dir, std::string& who, std::string& since) {
    const auto j = readJson(dir / "checkout.lock");
    if (!j) return false;
    who = (*j)["user"].asString("someone");
    since = (*j)["since"].asString();
    return true;
}

VaultItem loadItem(const fs::path& dir) {
    VaultItem item;
    item.key = dir.filename().string();
    if (const auto j = readJson(dir / "item.json")) item.description = (*j)["description"].asString();
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(dir, ec)) {
        const std::string f = e.path().filename().string();
        if (f.rfind("rev-", 0) != 0 || e.path().extension() != ".json") continue;
        const auto j = readJson(e.path());
        if (!j) continue;
        VaultRevision r;
        r.revision = (*j)["revision"].asString();
        r.date = (*j)["date"].asString();
        r.author = (*j)["author"].asString();
        r.note = (*j)["note"].asString();
        r.state = parseReleaseState((*j)["state"].asString()).value_or(ReleaseState::InWork);
        r.file = (dir / ("rev-" + vaultKey(r.revision) + ".sicdm")).string();
        if (!r.revision.empty() && fs::exists(r.file, ec)) item.revisions.push_back(r);
    }
    // Oldest first: B after A, AA after Z, 10 after 9.
    std::sort(item.revisions.begin(), item.revisions.end(), [](const VaultRevision& x, const VaultRevision& y) {
        if (x.revision.size() != y.revision.size()) return x.revision.size() < y.revision.size();
        return x.revision < y.revision;
    });
    readLock(dir, item.lockedBy, item.lockedSince);
    return item;
}

} // namespace

std::vector<VaultItem> vaultItems(const std::string& vault, std::string* why) {
    std::vector<VaultItem> out;
    std::error_code ec;
    if (vault.empty() || !fs::is_directory(vault, ec)) {
        if (why) *why = vault.empty() ? "No vault folder (VAULT FOLDER path sets it)" : "The vault folder " + vault + " isn't there";
        return out;
    }
    for (const auto& e : fs::directory_iterator(vault, ec))
        if (e.is_directory(ec) && e.path().filename().string().front() != '.') out.push_back(loadItem(e.path()));
    std::sort(out.begin(), out.end(), [](const VaultItem& x, const VaultItem& y) { return x.key < y.key; });
    return out;
}

std::optional<VaultItem> vaultItem(const std::string& vault, const std::string& key, std::string* why) {
    std::error_code ec;
    const fs::path dir = fs::path(vault) / vaultKey(key);
    if (vault.empty() || !fs::is_directory(dir, ec)) {
        if (why) *why = vault.empty() ? "No vault folder (VAULT FOLDER path sets it)" : "No item " + key + " in the vault";
        return std::nullopt;
    }
    return loadItem(dir);
}

bool vaultLock(const std::string& vault, const std::string& key, std::string* why) {
    std::error_code ec;
    const fs::path dir = fs::path(vault) / vaultKey(key);
    if (vault.empty() || !fs::is_directory(vault, ec)) {
        if (why) *why = "No vault folder (VAULT FOLDER path sets it)";
        return false;
    }
    fs::create_directories(dir, ec);
    const std::string me = currentUser();
    std::string who, since;
    if (readLock(dir, who, since)) {
        if (who == me) return true; // already ours
        if (why) *why = key + " is checked out by " + who + (since.empty() ? std::string() : " since " + since);
        return false;
    }
    // Made only if it isn't there: of two people checking out at once, one wins.
    const std::string path = (dir / "checkout.lock").string();
    std::FILE* f = std::fopen(path.c_str(), "wx");
    if (!f) {
        if (readLock(dir, who, since) && who != me) {
            if (why) *why = key + " is checked out by " + who;
            return false;
        }
        if (why) *why = "Cannot write the lock in " + dir.string();
        return false;
    }
    const std::string body = Json::object().set("user", me).set("since", todayIso()).dump();
    const bool ok = std::fwrite(body.data(), 1, body.size(), f) == body.size();
    std::fclose(f);
    if (!ok && why) *why = "Cannot write the lock in " + dir.string();
    return ok;
}

bool vaultUnlock(const std::string& vault, const std::string& key, bool force, std::string* why) {
    const fs::path dir = fs::path(vault) / vaultKey(key);
    std::string who, since;
    if (!readLock(dir, who, since)) return true; // nobody has it
    if (who != currentUser() && !force) {
        if (why) *why = key + " is checked out by " + who + ", not you (VAULT UNLOCK " + key + " FORCE takes their lock away)";
        return false;
    }
    std::error_code ec;
    fs::remove(dir / "checkout.lock", ec);
    if (ec && why) *why = "Cannot remove the lock: " + ec.message();
    return !ec;
}

bool vaultCheckIn(const std::string& vault, const std::string& key, const std::string& projectFile, const ProductData& product, const std::string& note,
                  std::string* why) {
    std::error_code ec;
    if (vault.empty() || !fs::is_directory(vault, ec)) {
        if (why) *why = "No vault folder (VAULT FOLDER path sets it)";
        return false;
    }
    const fs::path dir = fs::path(vault) / vaultKey(key);
    fs::create_directories(dir, ec);
    std::string who, since;
    const bool locked = readLock(dir, who, since);
    if (locked && who != currentUser()) {
        if (why) *why = key + " is checked out by " + who + ": they check it in";
        return false;
    }
    if (!locked && !loadItem(dir).revisions.empty()) { // a new item needs no checkout; a known one does
        if (why) *why = key + " is in the vault: VAULT CHECKOUT " + key + " first, so no one else changes it at the same time";
        return false;
    }
    const std::string rev = product.revision.empty() ? "A" : product.revision;
    const fs::path meta = dir / ("rev-" + vaultKey(rev) + ".json"), file = dir / ("rev-" + vaultKey(rev) + ".sicdm");
    if (const auto old = readJson(meta)) {
        const auto st = parseReleaseState((*old)["state"].asString());
        if (st && (*st == ReleaseState::Released || *st == ReleaseState::Obsolete)) {
            if (why) *why = "Revision " + rev + " of " + key + " is " + releaseStateName(*st) + " in the vault: REVISION NEXT starts the next one";
            return false;
        }
    }
    std::ifstream in(projectFile, std::ios::binary);
    if (!in) {
        if (why) *why = "Cannot read " + projectFile;
        return false;
    }
    const std::string data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (!writeFile(file.string(), data, why)) return false;
    const Json record = Json::object()
                            .set("revision", rev)
                            .set("state", releaseStateName(product.state))
                            .set("date", todayIso())
                            .set("author", currentUser())
                            .set("note", note);
    if (!writeFile(meta.string(), record.dump(), why)) return false;
    const auto d = product.properties.find("DESCRIPTION");
    Json itemJson = Json::object().set("key", key).set("description", d == product.properties.end() ? std::string() : d->second);
    if (const auto n = product.properties.find("NUMBER"); n != product.properties.end()) itemJson.set("number", n->second);
    if (!writeFile((dir / "item.json").string(), itemJson.dump(), why)) return false;
    fs::remove(dir / "checkout.lock", ec); // checked in: free for the next person
    return true;
}

} // namespace sicdm

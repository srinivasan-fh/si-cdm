#include "core/Recovery.hpp"

#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Json.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <cerrno>
#include <csignal>
#include <unistd.h>
#endif

namespace fs = std::filesystem;

namespace sicdm {

namespace {

long currentPid() {
#if defined(_WIN32)
    return static_cast<long>(GetCurrentProcessId());
#else
    return static_cast<long>(getpid());
#endif
}

bool processRunning(long pid) {
    if (pid <= 0) return false;
    if (pid == currentPid()) return true;
#if defined(_WIN32)
    HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, static_cast<DWORD>(pid));
    if (!h) return false;
    DWORD code = 0;
    const bool alive = GetExitCodeProcess(h, &code) && code == STILL_ACTIVE;
    CloseHandle(h);
    return alive;
#else
    return kill(static_cast<pid_t>(pid), 0) == 0 || errno == EPERM;
#endif
}

std::string env(const char* name) {
    const char* v = std::getenv(name);
    return v ? std::string(v) : std::string();
}

std::int64_t unixNow() {
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// Copies are named <pid>-<id>.sicdm with a <pid>-<id>.json beside it.
long pidOf(const fs::path& p) {
    const std::string s = p.stem().string();
    const std::size_t dash = s.find('-');
    if (dash == std::string::npos || dash == 0) return -1;
    char* end = nullptr;
    const long pid = std::strtol(s.c_str(), &end, 10);
    return end == s.c_str() + dash ? pid : -1;
}

std::string readAll(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

std::string recoveryDirectory() {
    if (const std::string d = env("SICDM_RECOVERY_DIR"); !d.empty()) return d;
#if defined(_WIN32)
    if (const std::string d = env("LOCALAPPDATA"); !d.empty()) return (fs::path(d) / "SI-CDM" / "Recovery").string();
#elif defined(__APPLE__)
    if (const std::string h = env("HOME"); !h.empty()) return (fs::path(h) / "Library" / "Application Support" / "SI-CDM" / "Recovery").string();
#else
    if (const std::string d = env("XDG_STATE_HOME"); !d.empty()) return (fs::path(d) / "sicdm" / "recovery").string();
    if (const std::string h = env("HOME"); !h.empty()) return (fs::path(h) / ".local" / "state" / "sicdm" / "recovery").string();
#endif
    std::error_code ec;
    return (fs::temp_directory_path(ec) / "sicdm-recovery").string();
}

std::vector<RecoveryFile> findRecoveryFiles(const std::string& directory) {
    std::vector<RecoveryFile> out;
    std::error_code ec;
    for (fs::directory_iterator it(directory, ec), end; !ec && it != end; it.increment(ec)) {
        const fs::path p = it->path();
        if (p.extension() != ".json") continue;
        const long pid = pidOf(p);
        if (pid < 0 || processRunning(pid)) continue; // its session is still going
        fs::path project = p;
        project.replace_extension(".sicdm");
        if (!fs::exists(project, ec)) continue;
        const auto meta = Json::parse(readAll(p));
        if (!meta || !meta->isObject()) continue;
        RecoveryFile f;
        f.project = project.string();
        f.meta = p.string();
        f.original = (*meta)["original"].asString("");
        f.name = (*meta)["name"].asString("Untitled");
        f.savedAt = static_cast<std::int64_t>((*meta)["saved_at"].asNumber(0));
        out.push_back(std::move(f));
    }
    std::sort(out.begin(), out.end(), [](const RecoveryFile& a, const RecoveryFile& b) { return a.savedAt > b.savedAt; });
    return out;
}

void discardRecoveryFile(const RecoveryFile& f) {
    std::error_code ec;
    fs::remove(f.project, ec);
    fs::remove(f.meta, ec);
}

void adoptRecoveryFile(const RecoveryFile& f, const std::string& directory) {
    // Renamed under this session's process id with an id no document uses (negative).
    std::error_code ec;
    for (int k = 1; k < 100000; ++k) {
        const fs::path stem = fs::path(directory) / (std::to_string(currentPid()) + "-r" + std::to_string(k));
        if (fs::exists(stem.string() + ".json", ec)) continue;
        fs::rename(f.project, stem.string() + ".sicdm", ec);
        fs::rename(f.meta, stem.string() + ".json", ec);
        return;
    }
}

Autosaver::Autosaver(std::string directory, double intervalSeconds) : m_dir(std::move(directory)), m_interval(intervalSeconds) {}

std::string Autosaver::projectPath(int id) const { return (fs::path(m_dir) / (std::to_string(currentPid()) + "-" + std::to_string(id) + ".sicdm")).string(); }

bool Autosaver::update(const Document& doc, int id, std::uint64_t signature, double now, std::string* error) {
    Entry& e = m_entries[id];
    if (!doc.dirty) { // saved (or never changed): no copy needed
        if (e.written != 0) forget(id);
        m_entries[id] = Entry{};
        return false;
    }
    if (signature == e.written) return false;
    if (e.pendingSince < 0) e.pendingSince = now;
    if (now - e.pendingSince < m_interval) return false;
    const std::string project = projectPath(id);
    if (!saveProject(doc.scene, doc.state, project, error)) {
        e.pendingSince = now; // try again after another interval
        return false;
    }
    Json meta = Json::object();
    meta.set("original", doc.path).set("name", doc.displayName()).set("saved_at", static_cast<std::int64_t>(unixNow()));
    fs::path mp(project);
    mp.replace_extension(".json");
    if (!writeFile(mp.string(), meta.dump() + "\n", error)) return false;
    e.written = signature;
    e.pendingSince = -1;
    return true;
}

double Autosaver::nextDue() const {
    double due = -1;
    for (const auto& [id, e] : m_entries)
        if (e.pendingSince >= 0 && (due < 0 || e.pendingSince + m_interval < due)) due = e.pendingSince + m_interval;
    return due;
}

void Autosaver::forget(int id) {
    std::error_code ec;
    fs::path p(projectPath(id));
    fs::remove(p, ec);
    p.replace_extension(".json");
    fs::remove(p, ec);
    m_entries.erase(id);
}

void Autosaver::clear() {
    std::error_code ec;
    const std::string prefix = std::to_string(currentPid()) + "-";
    std::vector<fs::path> mine;
    for (fs::directory_iterator it(m_dir, ec), end; !ec && it != end; it.increment(ec))
        if (it->path().filename().string().rfind(prefix, 0) == 0) mine.push_back(it->path());
    for (const fs::path& p : mine) fs::remove(p, ec);
    m_entries.clear();
}

} // namespace sicdm

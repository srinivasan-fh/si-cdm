#include "core/UpdateCheck.hpp"

#include "core/Json.hpp"
#include "core/Plugin.hpp"
#include "core/Sha256.hpp"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>
#else
#include <sys/wait.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

namespace sicdm {

namespace {

// "1.10.2-rc1" -> {1, 10, 2}, "rc1".
void splitVersion(const std::string& v, std::vector<long>& parts, std::string& suffix) {
    std::size_t i = 0;
    if (i < v.size() && (v[i] == 'v' || v[i] == 'V')) ++i;
    while (i < v.size() && std::isdigit(static_cast<unsigned char>(v[i]))) {
        long n = 0;
        while (i < v.size() && std::isdigit(static_cast<unsigned char>(v[i]))) n = n * 10 + (v[i++] - '0');
        parts.push_back(n);
        if (i < v.size() && v[i] == '.') ++i;
        else break;
    }
    if (i < v.size() && (v[i] == '-' || v[i] == '+')) ++i;
    suffix = v.substr(i);
}

// file:///C:/x or file:///home/x -> a path; percent escapes decoded.
std::string filePath(const std::string& url) {
    std::string p = url.substr(7);
    if (p.rfind("localhost/", 0) == 0) p = p.substr(9);
#if defined(_WIN32)
    if (p.size() > 2 && p[0] == '/' && p[2] == ':') p = p.substr(1);
#endif
    std::string out;
    for (std::size_t i = 0; i < p.size(); ++i) {
        if (p[i] == '%' && i + 2 < p.size() && std::isxdigit(static_cast<unsigned char>(p[i + 1])) && std::isxdigit(static_cast<unsigned char>(p[i + 2]))) {
            out += static_cast<char>(std::stoi(p.substr(i + 1, 2), nullptr, 16));
            i += 2;
        } else {
            out += p[i];
        }
    }
    return out;
}

} // namespace

int compareVersions(const std::string& a, const std::string& b) {
    std::vector<long> pa, pb;
    std::string sa, sb;
    splitVersion(a, pa, sa);
    splitVersion(b, pb, sb);
    for (std::size_t i = 0; i < std::max(pa.size(), pb.size()); ++i) {
        const long x = i < pa.size() ? pa[i] : 0, y = i < pb.size() ? pb[i] : 0;
        if (x != y) return x < y ? -1 : 1;
    }
    if (sa == sb) return 0;
    if (sa.empty()) return 1; // a release is newer than its pre-releases
    if (sb.empty()) return -1;
    return sa < sb ? -1 : 1;
}

bool parseReleaseFeed(const std::string& text, ReleaseInfo& out, std::string* error) {
    std::string perr;
    const auto j = Json::parse(text, &perr);
    if (!j || !j->isObject()) {
        if (error) *error = "The release feed isn't JSON" + (perr.empty() ? std::string() : ": " + perr);
        return false;
    }
    out = {};
    out.version = (*j)["version"].asString(std::string());
    std::vector<long> parts;
    std::string suffix;
    splitVersion(out.version, parts, suffix);
    if (parts.empty()) {
        if (error) *error = "The release feed has no version";
        return false;
    }
    out.date = (*j)["date"].asString(std::string());
    out.notes = (*j)["notes"].asString(std::string());
    out.download = (*j)["url"].asString(std::string());
#if defined(_WIN32)
    const char* platform = "windows";
#elif defined(__APPLE__)
    const char* platform = "macos";
#else
    const char* platform = "linux";
#endif
    if (const std::string d = (*j)["downloads"][platform].asString(std::string()); !d.empty()) out.download = d;
    const Json& sum = (*j)["sha256"];
    std::string digest = sum.isString() ? sum.asString() : sum[platform].asString(std::string());
    for (char& c : digest) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (digest.size() == 64 && digest.find_first_not_of("0123456789abcdef") == std::string::npos) out.sha256 = digest;
    return true;
}

bool fetchUrl(const std::string& url, std::string& body, std::string* error, double timeoutSeconds) {
    body.clear();
    auto fail = [&](const std::string& why) {
        if (error) *error = why;
        return false;
    };
    if (url.rfind("file://", 0) == 0) {
        std::ifstream f(filePath(url), std::ios::binary);
        if (!f) return fail("Can't read " + filePath(url));
        std::ostringstream s;
        s << f.rdbuf();
        body = s.str();
        return true;
    }
    if (url.rfind("https://", 0) != 0 && url.rfind("http://", 0) != 0) return fail("The address must start with https://, http:// or file://");
    for (char c : url)
        if (std::isspace(static_cast<unsigned char>(c)) || std::iscntrl(static_cast<unsigned char>(c))) return fail("The address has a space in it");
    // curl runs without a shell, so nothing in the address is interpreted; it starts with
    // http, so it can't be read as an option.
    const std::string seconds = std::to_string(static_cast<int>(timeoutSeconds));
    std::string err;
    const int code = runProgram({"curl", "--silent", "--show-error", "--fail", "--location", "--max-time", seconds, "--max-filesize", "1048576", url},
                                timeoutSeconds + 5.0, body, &err);
    if (code != 0) {
        body.clear();
        if (code == -1 && err.find("Can't start") != std::string::npos) return fail("The update check needs the curl program, and it wasn't found");
        return fail("Couldn't fetch " + url + (err.empty() ? std::string() : ": " + err));
    }
    return true;
}

std::string UpdateResult::describe(const std::string& current) const {
    if (!ok) return "Update check failed: " + error;
    if (!newer) return "SI CDM " + current + " is up to date (the latest is " + latest.version + ")";
    std::string s = "SI CDM " + latest.version + " is available (this is " + current + ")";
    if (!latest.date.empty()) s += ", released " + latest.date;
    if (!latest.download.empty()) s += ": " + latest.download;
    return s;
}

UpdateResult checkForUpdate(const std::string& url, const std::string& current) {
    UpdateResult r;
    if (url.empty()) {
        r.error = "No release feed is set (PREFERENCES SET update-url https://...)";
        return r;
    }
    std::string body;
    if (!fetchUrl(url, body, &r.error)) return r;
    if (!parseReleaseFeed(body, r.latest, &r.error)) return r;
    r.ok = true;
    r.newer = compareVersions(r.latest.version, current) > 0;
    return r;
}

namespace {

std::atomic<bool> g_installerStarted{false};

std::string envOr(const char* k) {
    const char* v = std::getenv(k);
    return v ? v : "";
}

// The file name at the end of an address ("" when it names a folder or page).
std::string fileNameOf(const std::string& url) {
    std::string u = url.substr(0, url.find_first_of("?#"));
    const std::size_t slash = u.find_last_of('/');
    std::string name = slash == std::string::npos ? u : u.substr(slash + 1);
    std::string lower = name;
    for (char& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    static const char* kinds[] = {".exe", ".msi", ".dmg", ".pkg", ".deb", ".rpm", ".appimage", ".zip", ".tar.gz", ".tgz", ".tar.xz"};
    for (const char* k : kinds) {
        const std::string e = k;
        if (lower.size() > e.size() && lower.compare(lower.size() - e.size(), e.size(), e) == 0) {
            for (char c : name)
                if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '-' || c == '_' || c == '+')) return "";
            return name;
        }
    }
    return "";
}

} // namespace

std::string downloadsFolder() {
    namespace fs = std::filesystem;
    std::error_code ec;
#if defined(_WIN32)
    const std::string home = envOr("USERPROFILE");
#else
    const std::string home = envOr("HOME");
#endif
    if (!home.empty() && fs::is_directory(fs::path(home) / "Downloads", ec)) return (fs::path(home) / "Downloads").string();
    return fs::temp_directory_path(ec).string();
}

bool downloadRelease(const ReleaseInfo& release, const std::string& dirIn, std::string& path, bool& checked, std::string* error) {
    namespace fs = std::filesystem;
    auto fail = [&](const std::string& why) {
        if (error) *error = why;
        return false;
    };
    checked = false;
    const std::string name = fileNameOf(release.download);
    if (name.empty())
        return fail("The release feed gives a page for this computer, not an installer to download: " + release.download);
    const std::string dir = dirIn.empty() ? downloadsFolder() : dirIn;
    std::error_code ec;
    fs::create_directories(dir, ec);
    path = (fs::path(dir) / name).string();
    const std::string part = path + ".part";
    fs::remove(part, ec);
    if (release.download.rfind("file://", 0) == 0) {
        fs::copy_file(filePath(release.download), part, fs::copy_options::overwrite_existing, ec);
        if (ec) return fail("Can't copy " + filePath(release.download) + ": " + ec.message());
    } else {
        if (release.download.rfind("https://", 0) != 0 && release.download.rfind("http://", 0) != 0)
            return fail("The download must be an https://, http:// or file:// address");
        for (char c : release.download)
            if (std::isspace(static_cast<unsigned char>(c)) || std::iscntrl(static_cast<unsigned char>(c))) return fail("The download address has a space in it");
        std::string out, err;
        const int code = runProgram({"curl", "--silent", "--show-error", "--fail", "--location", "--max-time", "1800", "--output", part, release.download},
                                    1810.0, out, &err);
        if (code != 0) {
            fs::remove(part, ec);
            return fail("Couldn't download " + release.download + (err.empty() ? std::string() : ": " + err));
        }
    }
    if (!release.sha256.empty()) {
        const auto got = sha256File(part);
        if (!got || *got != release.sha256) {
            fs::remove(part, ec);
            return fail("The download doesn't match the release feed's SHA-256 (it may be damaged or not the real one), so it was deleted");
        }
        checked = true;
    }
    fs::remove(path, ec);
    fs::rename(part, path, ec);
    if (ec) return fail("Can't write " + path + ": " + ec.message());
    return true;
}

bool openWithSystem(const std::string& path, std::string* error) {
#if defined(_WIN32)
    const int n = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, nullptr, 0);
    std::wstring w(static_cast<std::size_t>(n > 0 ? n : 1), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, w.data(), n);
    const auto r = reinterpret_cast<INT_PTR>(ShellExecuteW(nullptr, L"open", w.c_str(), nullptr, nullptr, SW_SHOWNORMAL));
    if (r <= 32) {
        if (error) *error = "Windows couldn't open " + path;
        return false;
    }
    return true;
#else
#if defined(__APPLE__)
    const char* opener = "open";
#else
    const char* opener = "xdg-open";
#endif
    // Started and left to run: a second fork so nothing waits for it.
    const pid_t pid = fork();
    if (pid < 0) {
        if (error) *error = "Can't start " + std::string(opener);
        return false;
    }
    if (pid == 0) {
        if (fork() == 0) {
            setsid();
            execlp(opener, opener, path.c_str(), static_cast<char*>(nullptr));
            _exit(127);
        }
        _exit(0);
    }
    int status = 0;
    waitpid(pid, &status, 0);
    return true;
#endif
}

bool takeInstallerStarted() { return g_installerStarted.exchange(false); }

void noteInstallerStarted() { g_installerStarted = true; }

} // namespace sicdm

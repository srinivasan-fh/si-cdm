#include "core/UpdateCheck.hpp"

#include "core/Json.hpp"
#include "core/Plugin.hpp"

#include <algorithm>
#include <cctype>
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

} // namespace sicdm

// Licence files: the signed message, checking against the build's key, and the
// installed licence.

#include "core/Licence.hpp"

#include "core/Ed25519.hpp"
#include "core/FileIO.hpp"
#include "core/Json.hpp"
#include "core/Preferences.hpp"

#include <ctime>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#ifndef SICDM_LICENCE_PUBLIC_KEY
#define SICDM_LICENCE_PUBLIC_KEY ""
#endif

namespace sicdm {

namespace fs = std::filesystem;

std::string licenceMessage(const Licence& l) {
    return "SI CDM licence 1\nlicensee=" + l.licensee + "\nemail=" + l.email + "\nedition=" + l.edition + "\nseats=" + std::to_string(l.seats) +
           "\nissued=" + l.issued + "\nexpires=" + l.expires + "\n";
}

Json licenceToJson(const Licence& l) {
    Json j = Json::object();
    j.set("format", "sicdm-licence").set("version", 1);
    j.set("licensee", l.licensee).set("email", l.email).set("edition", l.edition).set("seats", l.seats);
    j.set("issued", l.issued).set("expires", l.expires).set("signature", l.signature);
    return j;
}

bool licenceFromJson(const Json& j, Licence& out, std::string* error) {
    if (!j.isObject() || j["format"].asString(std::string()) != "sicdm-licence") {
        if (error) *error = "Not an SI CDM licence file";
        return false;
    }
    if (j["version"].asInt(0) != 1) {
        if (error) *error = "A licence of a newer kind: update SI CDM";
        return false;
    }
    out = Licence{};
    out.licensee = j["licensee"].asString(std::string());
    out.email = j["email"].asString(std::string());
    out.edition = j["edition"].asString(std::string());
    out.seats = j["seats"].asInt(1);
    out.issued = j["issued"].asString(std::string());
    out.expires = j["expires"].asString(std::string());
    out.signature = j["signature"].asString(std::string());
    if (out.licensee.empty() || out.signature.empty()) {
        if (error) *error = "The licence has no licensee or no signature";
        return false;
    }
    return true;
}

std::string LicenceStatus::describe() const {
    const Licence& l = licence;
    const std::string who = l.licensee + (l.email.empty() ? "" : " <" + l.email + ">");
    const std::string what = l.edition + ", " + std::to_string(l.seats) + (l.seats == 1 ? " seat" : " seats") +
                             (l.expires.empty() ? std::string() : ", until " + l.expires);
    switch (state) {
    case LicenceState::Valid: return "Licensed to " + who + " (" + what + ")";
    case LicenceState::Expired: return "The licence for " + who + " expired on " + l.expires + ": running as the Community edition";
    case LicenceState::BadSignature: return "The licence for " + who + " doesn't check out (a changed file, or another vendor's): running as the Community edition";
    case LicenceState::NoKey: return "Community edition (this build checks no licences)";
    case LicenceState::None: break;
    }
    return "Community edition";
}

LicenceStatus checkLicence(const Licence& l, const std::string& publicKeyHex, const std::string& today) {
    LicenceStatus st;
    st.licence = l;
    Ed25519PublicKey key{};
    if (publicKeyHex.empty() || !fromHex(publicKeyHex, key.data(), key.size())) {
        st.state = LicenceState::NoKey;
        return st;
    }
    Ed25519Signature sig{};
    const std::string m = licenceMessage(l);
    if (!fromHex(l.signature, sig.data(), sig.size()) || !ed25519Verify(key, reinterpret_cast<const std::uint8_t*>(m.data()), m.size(), sig)) {
        st.state = LicenceState::BadSignature;
        return st;
    }
    st.state = !l.expires.empty() && today > l.expires ? LicenceState::Expired : LicenceState::Valid; // ISO dates compare as text
    return st;
}

bool signLicence(Licence& l, const std::string& seedHex) {
    Ed25519Seed seed{};
    if (!fromHex(seedHex, seed.data(), seed.size())) return false;
    const std::string m = licenceMessage(l);
    const Ed25519Signature sig = ed25519Sign(seed, reinterpret_cast<const std::uint8_t*>(m.data()), m.size());
    l.signature = toHex(sig.data(), sig.size());
    return true;
}

std::string buildLicenceKey() { return SICDM_LICENCE_PUBLIC_KEY; }

std::string todayIso() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    char b[16];
    std::strftime(b, sizeof b, "%Y-%m-%d", &tm);
    return b;
}

namespace {

std::string env(const char* k) {
    const char* v = std::getenv(k);
    return v ? v : "";
}

bool readLicence(const std::string& path, Licence& l, std::string* error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        if (error) *error = "Can't read " + path;
        return false;
    }
    std::ostringstream s;
    s << f.rdbuf();
    std::string perr;
    const auto j = Json::parse(s.str(), &perr);
    if (!j) {
        if (error) *error = path + " isn't JSON: " + perr;
        return false;
    }
    return licenceFromJson(*j, l, error);
}

} // namespace

std::string licencePath() {
    if (const std::string p = env("SICDM_LICENCE"); !p.empty()) return p;
    return (fs::path(configDirectory()) / "licence.json").string();
}

LicenceStatus currentLicence() {
    LicenceStatus st;
    st.path = licencePath();
    std::error_code ec;
    if (!fs::exists(st.path, ec)) {
        st.state = buildLicenceKey().empty() ? LicenceState::NoKey : LicenceState::None;
        return st;
    }
    Licence l;
    if (!readLicence(st.path, l, nullptr)) {
        st.state = LicenceState::BadSignature;
        return st;
    }
    LicenceStatus c = checkLicence(l, buildLicenceKey(), todayIso());
    c.path = st.path;
    return c;
}

bool installLicence(const std::string& file, LicenceStatus* status, std::string* error) {
    Licence l;
    if (!readLicence(file, l, error)) return false;
    const LicenceStatus st = checkLicence(l, buildLicenceKey(), todayIso());
    if (status) *status = st;
    if (st.state == LicenceState::BadSignature || st.state == LicenceState::NoKey) {
        if (error) *error = st.describe();
        return false;
    }
    const std::string to = licencePath();
    std::error_code ec;
    fs::create_directories(fs::path(to).parent_path(), ec);
    if (!writeFile(to, licenceToJson(l).dump() + "\n", error)) return false;
    if (status) status->path = to;
    return true;
}

bool removeLicence(std::string* error) {
    std::error_code ec;
    const std::string p = licencePath();
    if (!fs::exists(p, ec)) {
        if (error) *error = "No licence is installed";
        return false;
    }
    fs::remove(p, ec);
    if (ec) {
        if (error) *error = "Can't remove " + p + ": " + ec.message();
        return false;
    }
    return true;
}

} // namespace sicdm

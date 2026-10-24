#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/CrashReport.hpp"
#include "core/Document.hpp"
#include "core/Ed25519.hpp"
#include "core/Json.hpp"
#include "core/Licence.hpp"
#include "core/Notices.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

bool has(const std::string& s, const std::string& what) { return s.find(what) != std::string::npos; }

std::string slurp(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream s;
    s << f.rdbuf();
    return s.str();
}

std::string text(const CommandOutput& r) {
    std::string s;
    for (const std::string& l : r.lines) s += l + "\n";
    return s;
}

void setEnv(const char* name, const std::string& value) {
#if defined(_WIN32)
    _putenv_s(name, value.c_str());
#else
    setenv(name, value.c_str(), 1);
#endif
}

std::string quoted(const std::string& s) { return "\"" + s + "\""; }

// Runs a program; on Windows cmd.exe wants the whole line quoted again.
int runProgram(const std::string& line) {
#if defined(_WIN32)
    return std::system(("\"" + line + "\"").c_str());
#else
    return std::system(line.c_str());
#endif
}

// Runs a program that is meant to crash, without the shell announcing it.
int runCrashing(const std::string& line) {
#if defined(_WIN32)
    return runProgram(line);
#else
    return std::system(("{ " + line + "; } 2>/dev/null").c_str());
#endif
}

} // namespace

TEST_CASE("SHA-512 and Ed25519 match FIPS 180-4 and RFC 8032") {
    const auto h = sha512(std::string("abc"));
    CHECK(toHex(h.data(), h.size()) ==
          "ddaf35a193617abacc417349ae20413112e6fa4e89a97ea20a9eeee64b55d39a2192992a274fc1a836ba3c23a3feebbd454d4423643ce80e2a9ac94fa54ca49f");
    // Two blocks (the length doesn't fit after the padding of the first).
    const auto h2 = sha512(std::string(200, 'a'));
    CHECK(toHex(h2.data(), h2.size()) ==
          "4b11459c33f52a22ee8236782714c150a3b2c60994e9acee17fe68947a3e6789f31e7668394592da7bef827cddca88c4e6f86e4df7ed1ae6cba71f3e98faee9f");
    // RFC 8032, 7.1, tests 1 and 2.
    Ed25519Seed seed{};
    REQUIRE(fromHex("9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60", seed.data(), 32));
    const Ed25519PublicKey pk = ed25519PublicKey(seed);
    CHECK(toHex(pk.data(), 32) == "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a");
    Ed25519Signature sig = ed25519Sign(seed, nullptr, 0);
    CHECK(toHex(sig.data(), 64) ==
          "e5564300c360ac729086e2cc806e828a84877f1eb8e5d974d873e065224901555fb8821590a33bacc61e39701cf9b46bd25bf5f0595bbe24655141438e7a100b");
    CHECK(ed25519Verify(pk, nullptr, 0, sig));
    REQUIRE(fromHex("4ccd089b28ff96da9db6c346ec114e0f5b8a319f35aba624da8cf6ed4fb8a6fb", seed.data(), 32));
    const std::uint8_t m2[1] = {0x72};
    const Ed25519PublicKey pk2 = ed25519PublicKey(seed);
    CHECK(toHex(pk2.data(), 32) == "3d4017c3e843895a92b70aa74d1b7ebc9c982ccf2ec4968cc0cd55f12af4660c");
    sig = ed25519Sign(seed, m2, 1);
    CHECK(toHex(sig.data(), 64) ==
          "92a009a9f0d4cab8720e820b5f642540a2b27b5416503f8fb3762223ebdb69da085ac1e43e15996e458f3613d0f11d8c387b2eaeb4302aeeb00d291612bb0c00");
    CHECK(ed25519Verify(pk2, m2, 1, sig));
    // A changed message, signature or key fails.
    const std::uint8_t other[1] = {0x73};
    CHECK(!ed25519Verify(pk2, other, 1, sig));
    CHECK(!ed25519Verify(pk, m2, 1, sig));
    Ed25519Signature bad = sig;
    bad[40] ^= 0x10;
    CHECK(!ed25519Verify(pk2, m2, 1, bad));
    bad = sig;
    bad[63] = 0xff; // S past the group's order
    CHECK(!ed25519Verify(pk2, m2, 1, bad));
}

TEST_CASE("Licences: signed by the vendor, checked against the build's key, expiring, tamper-evident") {
    const std::string seed = "4ccd089b28ff96da9db6c346ec114e0f5b8a319f35aba624da8cf6ed4fb8a6fb";
    const std::string key = "3d4017c3e843895a92b70aa74d1b7ebc9c982ccf2ec4968cc0cd55f12af4660c";
    Licence l;
    l.licensee = "Ada Lovelace";
    l.email = "ada@example.com";
    l.edition = "Professional";
    l.seats = 3;
    l.issued = "2026-01-01";
    l.expires = "2027-01-01";
    REQUIRE(signLicence(l, seed));
    CHECK(l.signature.size() == 128);
    LicenceStatus st = checkLicence(l, key, "2026-10-24");
    CHECK(st.state == LicenceState::Valid);
    CHECK(st.licensed());
    CHECK(has(st.describe(), "Licensed to Ada Lovelace <ada@example.com> (Professional, 3 seats, until 2027-01-01)"));
    CHECK(checkLicence(l, key, "2027-01-02").state == LicenceState::Expired);
    CHECK(checkLicence(l, "", "2026-10-24").state == LicenceState::NoKey);
    // Through JSON and back, still good; a changed field isn't.
    Licence back;
    REQUIRE(licenceFromJson(licenceToJson(l), back));
    CHECK(checkLicence(back, key, "2026-10-24").licensed());
    back.seats = 30;
    CHECK(checkLicence(back, key, "2026-10-24").state == LicenceState::BadSignature);
    back = l;
    back.expires.clear();
    CHECK(checkLicence(back, key, "2026-10-24").state == LicenceState::BadSignature);
    // Another vendor's key.
    CHECK(checkLicence(l, "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a", "2026-10-24").state == LicenceState::BadSignature);
    std::string err;
    CHECK(!licenceFromJson(Json::object(), back, &err));
    CHECK(has(err, "Not an SI CDM licence"));
    // This test build has no key: the program runs as the Community edition and won't
    // install a licence it can't check.
    const std::string dir = (std::filesystem::temp_directory_path() / "sicdm licence test").string();
    std::filesystem::create_directories(dir);
    const std::string file = (std::filesystem::path(dir) / "issued.json").string();
    {
        std::ofstream f(file, std::ios::binary);
        f << licenceToJson(l).dump();
    }
    setEnv("SICDM_LICENCE", (std::filesystem::path(dir) / "installed.json").string());
    Document d;
    if (buildLicenceKey().empty()) {
        CHECK(has(text(executeCommand(d, "LICENCE", nullptr)), "Community edition"));
        const auto r = executeCommand(d, "LICENCE INSTALL \"" + file + "\"", nullptr);
        CHECK(!r.ok);
        CHECK(has(text(r), "checks no licences"));
    }
    CHECK(!executeCommand(d, "LICENCE REMOVE", nullptr).ok); // nothing installed
    CHECK(has(text(executeCommand(d, "NOTICES", nullptr)), "Dear ImGui"));
    CHECK(has(thirdPartyNotices(), "GLFW"));
    std::filesystem::remove_all(dir);
}

TEST_CASE("sicdm-licence makes a key pair, issues a licence and verifies it") {
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "sicdm licence tool";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    const std::string tool = SICDM_LICENCE_TOOL;
    const std::string keyFile = (dir / "vendor.key").string(), lic = (dir / "licence.json").string(), out = (dir / "out.txt").string();
    REQUIRE(runProgram(quoted(tool) + " keygen " + quoted(keyFile) + " > " + quoted(out)) == 0);
    const std::string said = slurp(out);
    const auto at = said.find("Public key: ");
    REQUIRE(at != std::string::npos);
    const std::string pub = said.substr(at + 12, 64);
    CHECK(runProgram(quoted(tool) + " keygen " + quoted(keyFile) + " > " + quoted(out) + " 2>&1") != 0); // never overwrites a key
    REQUIRE(runProgram(quoted(tool) + " issue " + quoted(keyFile) + " --licensee \"Grace Hopper\" --seats 2 --expires 2099-12-31 --out " + quoted(lic) + " > " + quoted(out)) == 0);
    CHECK(runProgram(quoted(tool) + " verify " + quoted(lic) + " --public " + pub + " > " + quoted(out)) == 0);
    CHECK(has(slurp(out), "Licensed to Grace Hopper (Professional, 2 seats, until 2099-12-31)"));
    // The key the tool made is a real Ed25519 key.
    std::string seedHex = slurp(keyFile);
    seedHex.resize(64);
    Ed25519Seed seed{};
    REQUIRE(fromHex(seedHex, seed.data(), 32));
    const Ed25519PublicKey pk = ed25519PublicKey(seed);
    CHECK(toHex(pk.data(), 32) == pub);
    // Another key doesn't verify it.
    CHECK(runProgram(quoted(tool) + " verify " + quoted(lic) + " --public d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a > " + quoted(out)) != 0);
    std::filesystem::remove_all(dir);
}

TEST_CASE("Crash reports: a fault or an uncaught exception leaves a report with the last command and the stack") {
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "sicdm crash test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    const std::string probe = SICDM_CRASH_PROBE;
    CHECK(runCrashing(quoted(probe) + " " + quoted(dir.string()) + " segv") != 0);
    std::vector<CrashReportFile> reports = crashReports(dir.string());
    REQUIRE(reports.size() == 1);
    CHECK(!reports[0].seen);
    CHECK(reports[0].time > 1700000000);
    std::string r = slurp(reports[0].path);
    CHECK(has(r, "SI CDM crash report"));
    CHECK(has(r, "version: probe"));
    CHECK(has(r, "fault: SIGSEGV"));
    CHECK(has(r, "last command: PROBE 42"));
    CHECK(has(r, "stack:"));
    // An exception main() catches last: its message, then the abort.
    CHECK(runCrashing(quoted(probe) + " " + quoted(dir.string()) + " caught") != 0);
    reports = crashReports(dir.string());
    REQUIRE(reports.size() == 2);
    auto anyHas = [&](const std::string& a, const std::string& b) {
        for (const CrashReportFile& f : crashReports(dir.string())) {
            const std::string t = slurp(f.path);
            if (has(t, a) && has(t, b)) return true;
        }
        return false;
    };
    CHECK(anyHas("exception: the probe's caught exception", "SIGABRT"));
    // One nobody catches: std::terminate keeps its message; on Windows the fault handler sees
    // it first and names it.
    CHECK(runCrashing(quoted(probe) + " " + quoted(dir.string()) + " exception") != 0);
    REQUIRE(crashReports(dir.string()).size() == 3);
#if defined(_WIN32)
    CHECK(anyHas("uncaught C++ exception", "last command: PROBE 42") || anyHas("exception: the probe's exception", "SIGABRT"));
#else
    CHECK(anyHas("exception: the probe's exception", "SIGABRT"));
#endif
    // Shown once, then kept as seen; cleared on request.
    acknowledgeCrashReports(dir.string());
    reports = crashReports(dir.string());
    REQUIRE(reports.size() == 3);
    CHECK(reports[0].seen && reports[1].seen && reports[2].seen);
    clearCrashReports(dir.string());
    CHECK(crashReports(dir.string()).empty());
    std::filesystem::remove_all(dir);
}

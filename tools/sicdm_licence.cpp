// sicdm-licence: the vendor's tool for SI CDM licences.
//
//   sicdm-licence keygen vendor.key
//       a new Ed25519 key pair: the secret seed into vendor.key (keep it safe and out of
//       the source tree), the public key printed for the build:
//       cmake -DSICDM_LICENCE_PUBLIC_KEY=<public key> ...
//   sicdm-licence public vendor.key
//       the public key of a secret key file
//   sicdm-licence issue vendor.key --licensee "Name" [--email e] [--edition Professional]
//                 [--seats n] [--expires YYYY-MM-DD] [--issued YYYY-MM-DD] [--out licence.json]
//       a signed licence (to the screen without --out)
//   sicdm-licence verify licence.json --public <hex>
//       checks a licence against a public key

#include "core/Ed25519.hpp"
#include "core/FileIO.hpp"
#include "core/Json.hpp"
#include "core/Licence.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace sicdm;

namespace {

int usage() {
    std::fprintf(stderr,
                 "usage: sicdm-licence keygen vendor.key\n"
                 "       sicdm-licence public vendor.key\n"
                 "       sicdm-licence issue vendor.key --licensee NAME [--email E] [--edition E] [--seats N]\n"
                 "                     [--expires YYYY-MM-DD] [--issued YYYY-MM-DD] [--out licence.json]\n"
                 "       sicdm-licence verify licence.json --public HEX\n");
    return 2;
}

std::string readAll(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream s;
    s << f.rdbuf();
    return s.str();
}

bool readSeed(const std::string& path, std::string& hex) {
    hex = readAll(path);
    while (!hex.empty() && (hex.back() == '\n' || hex.back() == '\r' || hex.back() == ' ')) hex.pop_back();
    Ed25519Seed s{};
    if (!fromHex(hex, s.data(), s.size())) {
        std::fprintf(stderr, "%s doesn't hold a secret key (64 hex digits)\n", path.c_str());
        return false;
    }
    return true;
}

std::string publicOf(const std::string& seedHex) {
    Ed25519Seed s{};
    fromHex(seedHex, s.data(), s.size());
    const Ed25519PublicKey pk = ed25519PublicKey(s);
    return toHex(pk.data(), pk.size());
}

} // namespace

int main(int argc, char** argv) {
    std::vector<std::string> a(argv + 1, argv + argc);
    if (a.size() < 2) return usage();
    const std::string& cmd = a[0];
    if (cmd == "keygen") {
        std::ifstream exists(a[1]);
        if (exists) {
            std::fprintf(stderr, "%s exists: not overwriting a key\n", a[1].c_str());
            return 1;
        }
        Ed25519Seed seed{};
        if (!ed25519RandomSeed(seed)) {
            std::fprintf(stderr, "No random source\n");
            return 1;
        }
        const std::string hex = toHex(seed.data(), seed.size());
        std::string err;
        if (!writeFile(a[1], hex + "\n", &err)) {
            std::fprintf(stderr, "%s\n", err.c_str());
            return 1;
        }
        std::printf("Secret key written to %s: keep it safe, and out of the source tree.\n", a[1].c_str());
        std::printf("Public key: %s\n", publicOf(hex).c_str());
        std::printf("Build with: cmake -DSICDM_LICENCE_PUBLIC_KEY=%s ...\n", publicOf(hex).c_str());
        return 0;
    }
    if (cmd == "public") {
        std::string hex;
        if (!readSeed(a[1], hex)) return 1;
        std::printf("%s\n", publicOf(hex).c_str());
        return 0;
    }
    if (cmd == "issue") {
        std::string hex;
        if (!readSeed(a[1], hex)) return 1;
        Licence l;
        l.issued = todayIso();
        std::string out;
        for (std::size_t i = 2; i < a.size(); ++i) {
            const std::string& k = a[i];
            if (i + 1 >= a.size()) return usage();
            const std::string& v = a[++i];
            if (k == "--licensee") l.licensee = v;
            else if (k == "--email") l.email = v;
            else if (k == "--edition") l.edition = v;
            else if (k == "--seats") l.seats = std::max(1, std::atoi(v.c_str()));
            else if (k == "--expires") l.expires = v;
            else if (k == "--issued") l.issued = v;
            else if (k == "--out") out = v;
            else return usage();
        }
        if (l.licensee.empty()) return usage();
        for (const std::string* d : {&l.expires, &l.issued})
            if (!d->empty() && (d->size() != 10 || (*d)[4] != '-' || (*d)[7] != '-')) {
                std::fprintf(stderr, "Dates are YYYY-MM-DD\n");
                return 1;
            }
        signLicence(l, hex);
        const std::string text = licenceToJson(l).dump() + "\n";
        if (out.empty()) std::fputs(text.c_str(), stdout);
        else {
            std::string err;
            if (!writeFile(out, text, &err)) {
                std::fprintf(stderr, "%s\n", err.c_str());
                return 1;
            }
            std::printf("Wrote %s for %s\n", out.c_str(), l.licensee.c_str());
        }
        return 0;
    }
    if (cmd == "verify") {
        if (a.size() != 4 || a[2] != "--public") return usage();
        std::string perr;
        const auto j = Json::parse(readAll(a[1]), &perr);
        Licence l;
        std::string err;
        if (!j || !licenceFromJson(*j, l, &err)) {
            std::fprintf(stderr, "%s\n", j ? err.c_str() : perr.c_str());
            return 1;
        }
        const LicenceStatus st = checkLicence(l, a[3], todayIso());
        std::printf("%s\n", st.describe().c_str());
        return st.licensed() ? 0 : 1;
    }
    return usage();
}

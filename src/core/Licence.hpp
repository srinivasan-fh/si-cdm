#pragma once

// Licence files: who a copy of SI CDM is licensed to, signed by the vendor.
//
// A licence is a small JSON file whose fields are signed with the vendor's Ed25519 key.
// The program checks it against the public key it was built with (the CMake option
// SICDM_LICENCE_PUBLIC_KEY, 64 hex digits); the tool sicdm-licence makes key pairs and
// issues licences. A build without a key, or a copy without a licence, runs as the
// Community edition. Nothing is locked: the edition is shown, and code that wants to can
// ask for it with currentLicence().
//
// The signed message is the fields, one per line, in a fixed order:
//
//   SI CDM licence 1
//   licensee=<name>
//   email=<email>
//   edition=<edition>
//   seats=<n>
//   issued=<YYYY-MM-DD>
//   expires=<YYYY-MM-DD or empty: no end>

#include <string>

namespace sicdm {

class Json;

struct Licence {
    std::string licensee, email, edition = "Professional";
    int seats = 1;
    std::string issued, expires; // YYYY-MM-DD; expires empty: no end
    std::string signature;       // 128 hex digits
};

std::string licenceMessage(const Licence& l);
Json licenceToJson(const Licence& l);
bool licenceFromJson(const Json& j, Licence& out, std::string* error = nullptr);

enum class LicenceState { None, Valid, Expired, BadSignature, NoKey };

struct LicenceStatus {
    LicenceState state = LicenceState::None;
    Licence licence;
    std::string path; // where it was read from
    // "Licensed to Ada Lovelace (Professional, 1 seat)", "Community edition", ...
    std::string describe() const;
    bool licensed() const { return state == LicenceState::Valid; }
};

// Checks a licence against a public key (64 hex digits) on a day (YYYY-MM-DD).
LicenceStatus checkLicence(const Licence& l, const std::string& publicKeyHex, const std::string& today);

// Signs a licence with a seed (64 hex digits). False on a bad seed.
bool signLicence(Licence& l, const std::string& seedHex);

// The public key this build checks licences with (empty: none).
std::string buildLicenceKey();
// Today as YYYY-MM-DD (UTC).
std::string todayIso();

// $SICDM_LICENCE, else licence.json in the per-user configuration folder
// (%APPDATA%\SI-CDM, ~/Library/Application Support/SI-CDM, $XDG_CONFIG_HOME/sicdm).
std::string licencePath();
// The installed licence, checked (read again each call).
LicenceStatus currentLicence();
// Copies a licence file into place after checking it; false (and why) when it doesn't check out.
bool installLicence(const std::string& file, LicenceStatus* status, std::string* error);
bool removeLicence(std::string* error);

} // namespace sicdm

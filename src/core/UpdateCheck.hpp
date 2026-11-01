#pragma once

// Update check: is there a newer release than this one?
//
// The release feed is a small JSON file at an address the organisation chooses (the
// update-url preference), on a web server or a shared drive:
//
//   {
//     "version": "1.3.0",
//     "date": "2026-11-02",
//     "notes": "Lathe threading cycles; faster STEP import",
//     "url": "https://example.com/sicdm/1.3.0/",          the download page
//     "downloads": {"windows": "...", "macos": "...", "linux": "..."}   optional, per platform
//     "sha256": {"windows": "...", "macos": "...", "linux": "..."}      optional: the files' digests
//   }
//
// UPDATES INSTALL downloads this platform's file (an installer: .exe/.msi, .dmg/.pkg,
// .deb/.rpm/.AppImage or an archive), checks it against the digest when the feed gives
// one (a mismatch is refused and the file deleted), and opens it with the system: the
// Windows installer, the macOS disk image, the Linux package installer.
//
// file:// addresses are read directly; http:// and https:// ones are fetched with the
// system's curl program (part of Windows 10 and later, macOS and Linux), so SI CDM itself
// carries no network code. Nothing about the computer or its user is sent: it is a plain
// download of the feed.

#include <string>

namespace sicdm {

struct ReleaseInfo {
    std::string version, date, notes;
    std::string download; // for this platform when the feed has one, else its url
    std::string sha256;   // its digest when the feed gives one (lower-case hex)
};

// Compares dotted versions part by part as numbers ("1.10" > "1.9", "1.2" == "1.2.0"); a
// pre-release suffix ("1.3.0-rc1") is older than the release. <0, 0 or >0.
int compareVersions(const std::string& a, const std::string& b);

bool parseReleaseFeed(const std::string& text, ReleaseInfo& out, std::string* error = nullptr);

// The body at a file://, http:// or https:// address. False (and why) when it can't be had.
bool fetchUrl(const std::string& url, std::string& body, std::string* error = nullptr, double timeoutSeconds = 10.0);

struct UpdateResult {
    bool ok = false;    // the feed was read
    bool newer = false; // it names a newer release than `current`
    ReleaseInfo latest;
    std::string error;
    std::string describe(const std::string& current) const; // one line for the log
};

UpdateResult checkForUpdate(const std::string& url, const std::string& current);

// Downloads `release`'s file into `dir` (the system's downloads folder when empty) and
// checks its digest. `path` is where it went; `checked` whether a digest was compared.
bool downloadRelease(const ReleaseInfo& release, const std::string& dir, std::string& path, bool& checked, std::string* error = nullptr);
// The folder downloads go to: ~/Downloads (Windows: %USERPROFILE%\Downloads), else the temp folder.
std::string downloadsFolder();
// Opens a file with the program the system gives it (an installer runs), without waiting.
bool openWithSystem(const std::string& path, std::string* error = nullptr);
// Set when UPDATES INSTALL has opened an installer (the app then offers to close); cleared by reading.
bool takeInstallerStarted();
void noteInstallerStarted();

} // namespace sicdm

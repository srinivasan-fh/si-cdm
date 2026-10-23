#!/usr/bin/env bash
# Build, test and package SI CDM for release.
#
#   scripts/release.sh                  the platform this machine is (linux, macos or windows)
#   scripts/release.sh linux            .deb and .tar.gz                    (on Linux)
#   scripts/release.sh macos            .dmg                                (on macOS, Xcode)
#   scripts/release.sh windows          NSIS setup .exe and .zip            (on Windows, MSYS2 UCRT64 shell)
#   scripts/release.sh windows-cross    the Windows packages, cross-built   (on Linux, MinGW-w64)
#   scripts/release.sh all              every platform this machine can build
#                                       (Linux: linux + windows-cross; macOS: macos; Windows: windows)
#   scripts/release.sh version 1.2.0    set the version in CMakeLists.txt
#   scripts/release.sh tag              tag v<version> and push it: CI builds all three platforms
#                                       and publishes them as a GitHub release
#
# Options:
#   --clean        start from an empty build folder
#   --skip-tests   package without running the unit tests
#
# Packages land in dist/<version>/ with a SHA256SUMS-<platform>.txt for each platform.
#
# Signing, when the environment provides it (otherwise the packages are unsigned):
#   macOS    SICDM_CODESIGN_ID="Developer ID Application: ..."  signs the app (hardened runtime)
#            SICDM_NOTARY_PROFILE=<notarytool keychain profile>  notarizes and staples the .dmg
#   Windows  SICDM_WIN_CERT=<file.pfx> SICDM_WIN_CERT_PASS=...   signs the .exe files with
#            osslsigncode (MSYS2: pacman -S mingw-w64-ucrt-x86_64-osslsigncode; Linux: apt)
set -euo pipefail

cd "$(dirname "$0")/.."
ROOT=$(pwd)

die() { echo "release: $*" >&2; exit 1; }
step() { printf '\n==> %s\n' "$*"; }

VERSION=$(sed -n 's/^ *VERSION \([0-9][0-9.]*\)$/\1/p' CMakeLists.txt | head -1)
[[ -n "$VERSION" ]] || die "no VERSION in CMakeLists.txt's project()"

# A tag build must package the version it is tagged with.
if [[ "${GITHUB_REF_TYPE:-}" == tag && "${GITHUB_REF_NAME:-}" == v* && "$GITHUB_REF_NAME" != "v$VERSION" ]]; then
    die "tag $GITHUB_REF_NAME but CMakeLists.txt says $VERSION"
fi

case "$(uname -s)" in
    Linux*) HOST=linux ;;
    Darwin*) HOST=macos ;;
    MINGW* | MSYS* | CYGWIN*) HOST=windows ;;
    *) HOST=unknown ;;
esac

TARGET=""
CLEAN=false
TESTS=true
ARGS=()
for a in "$@"; do
    case "$a" in
        --clean) CLEAN=true ;;
        --skip-tests) TESTS=false ;;
        -h | --help) sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*) die "unknown option $a" ;;
        *) if [[ -z "$TARGET" ]]; then TARGET=$a; else ARGS+=("$a"); fi ;;
    esac
done
TARGET=${TARGET:-$HOST}

need() { command -v "$1" >/dev/null || die "$1 not found${2:+ ($2)}"; }

sha256() { if command -v sha256sum >/dev/null; then sha256sum "$@"; else shasum -a 256 "$@"; fi; }

# Copies the packages matching the globs from a build folder into dist/<version>/.
collect() {
    local platform=$1 dir=$2; shift 2
    local out="$ROOT/dist/$VERSION" files=()
    mkdir -p "$out"
    for g in "$@"; do
        for f in "$dir"/$g; do [[ -f "$f" ]] && files+=("$f"); done
    done
    ((${#files[@]})) || die "cpack wrote no packages in $dir"
    cp "${files[@]}" "$out/"
    (cd "$out" && sha256 "${files[@]##*/}" > "SHA256SUMS-$platform.txt")
    step "$platform packages in dist/$VERSION:"
    (cd "$out" && ls -l "${files[@]##*/}" "SHA256SUMS-$platform.txt")
}

fresh() { $CLEAN && rm -rf "$1"; return 0; }

win_sign() { # file...
    [[ -n "${SICDM_WIN_CERT:-}" ]] || return 0
    need osslsigncode
    for f in "$@"; do
        osslsigncode sign -pkcs12 "$SICDM_WIN_CERT" -pass "${SICDM_WIN_CERT_PASS:-}" -h sha256 \
            -n "SI CDM" -t http://timestamp.digicert.com -in "$f" -out "$f.signed"
        mv "$f.signed" "$f"
    done
}

release_linux() {
    [[ $HOST == linux ]] || die "linux packages are built on Linux"
    need cmake
    fresh build/release
    step "Linux $VERSION: configure and build"
    cmake --preset release
    cmake --build --preset release -j
    if $TESTS; then step "tests"; ctest --preset release; fi
    step "package"
    rm -f build/release/sicdm_*.deb build/release/sicdm-*.tar.gz
    cpack --preset release
    collect linux build/release "sicdm_*.deb" "sicdm-*.tar.gz"
}

release_macos() {
    [[ $HOST == macos ]] || die "macOS packages are built on macOS (Xcode)"
    need cmake; need xcodebuild "install Xcode"
    fresh build/xcode
    step "macOS $VERSION: configure and build (Universal)"
    cmake --preset xcode
    cmake --build --preset xcode-release -- -quiet
    if $TESTS; then step "tests"; ctest --preset xcode; fi
    if [[ -n "${SICDM_CODESIGN_ID:-}" ]]; then
        step "sign the app"
        codesign --force --options runtime --timestamp --sign "$SICDM_CODESIGN_ID" build/xcode/Release/sicdm-mcp
        codesign --force --deep --options runtime --timestamp --sign "$SICDM_CODESIGN_ID" "build/xcode/Release/SICDM.app"
    fi
    step "package"
    rm -f build/xcode/SICDM-*.dmg
    cpack --preset xcode
    local dmg
    dmg=$(ls build/xcode/SICDM-*.dmg | head -1)
    if [[ -n "${SICDM_CODESIGN_ID:-}" ]]; then codesign --force --timestamp --sign "$SICDM_CODESIGN_ID" "$dmg"; fi
    if [[ -n "${SICDM_NOTARY_PROFILE:-}" ]]; then
        step "notarize"
        xcrun notarytool submit "$dmg" --keychain-profile "$SICDM_NOTARY_PROFILE" --wait
        xcrun stapler staple "$dmg"
    fi
    collect macos build/xcode "SICDM-*.dmg"
}

release_windows() {
    [[ $HOST == windows ]] || die "use windows-cross on Linux; windows builds in an MSYS2 UCRT64 shell"
    need cmake "pacman -S mingw-w64-ucrt-x86_64-cmake"
    need makensis "pacman -S mingw-w64-ucrt-x86_64-nsis"
    fresh build/windows
    step "Windows $VERSION: configure and build"
    cmake --preset windows
    cmake --build --preset windows
    if $TESTS; then step "tests"; ctest --preset windows; fi
    win_sign build/windows/sicdm.exe build/windows/sicdm-mcp.exe
    step "package"
    rm -f build/windows/SICDM-*-windows-x64.*
    cpack --preset windows
    win_sign build/windows/SICDM-*-windows-x64.exe
    collect windows build/windows "SICDM-*-windows-x64.exe" "SICDM-*-windows-x64.zip"
}

release_windows_cross() {
    [[ $HOST == linux ]] || die "windows-cross runs on Linux"
    need cmake
    command -v x86_64-w64-mingw32-g++-posix >/dev/null || command -v x86_64-w64-mingw32-g++ >/dev/null ||
        die "MinGW-w64 not found (apt install g++-mingw-w64-x86-64-posix)"
    need makensis "apt install nsis"
    fresh build/mingw
    step "Windows $VERSION (cross-built): configure and build"
    cmake --preset mingw-cross
    cmake --build --preset mingw-cross -j
    if $TESTS; then
        local wine
        wine=$(command -v wine64 || ls /usr/lib/wine/wine64 2>/dev/null || command -v wine || true)
        if [[ -n "$wine" ]]; then
            step "tests (Wine)"
            (cd build/mingw/tests && WINEDEBUG=-all "$wine" ./sicdm_tests.exe > wine-tests.log 2>&1) || true
            grep -E '^\[FAIL|cases,' build/mingw/tests/wine-tests.log || true
            grep -q 'cases, 0 failed' build/mingw/tests/wine-tests.log ||
                die "tests failed under Wine (build/mingw/tests/wine-tests.log)"
        else
            echo "release: Wine not found, Windows tests skipped (apt install wine64)" >&2
        fi
    fi
    win_sign build/mingw/sicdm.exe build/mingw/sicdm-mcp.exe
    step "package"
    rm -f build/mingw/SICDM-*-windows-x64.*
    cpack --preset mingw-cross
    win_sign build/mingw/SICDM-*-windows-x64.exe
    collect windows build/mingw "SICDM-*-windows-x64.exe" "SICDM-*-windows-x64.zip"
}

set_version() {
    local v=${ARGS[0]:-}
    [[ "$v" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || die "usage: scripts/release.sh version X.Y.Z"
    sed -i.bak "s/^\( *VERSION \)$VERSION\$/\1$v/" CMakeLists.txt && rm -f CMakeLists.txt.bak
    grep -q "^ *VERSION $v\$" CMakeLists.txt || die "could not set the version"
    echo "Version $VERSION -> $v (commit CMakeLists.txt, then: scripts/release.sh tag)"
}

tag_release() {
    need git
    local tag="v$VERSION"
    [[ -z "$(git status --porcelain --untracked-files=no)" ]] || die "commit your changes first"
    git rev-parse -q --verify "refs/tags/$tag" >/dev/null && die "$tag already exists (scripts/release.sh version X.Y.Z)"
    git tag -a "$tag" -m "SI CDM $VERSION"
    git push origin "$tag"
    echo "Pushed $tag: CI builds macOS, Linux and Windows and publishes the GitHub release."
}

case "$TARGET" in
    linux) release_linux ;;
    macos) release_macos ;;
    windows) release_windows ;;
    windows-cross) release_windows_cross ;;
    all)
        case "$HOST" in
            linux) release_linux; release_windows_cross ;;
            macos) release_macos ;;
            windows) release_windows ;;
            *) die "unknown host" ;;
        esac
        echo "(macOS packages need a Mac, the others need Linux or Windows; CI builds all three: scripts/release.sh tag)"
        ;;
    version) set_version ;;
    tag) tag_release ;;
    *) die "unknown target $TARGET (linux | macos | windows | windows-cross | all | version | tag)" ;;
esac

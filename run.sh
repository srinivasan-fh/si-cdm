#!/usr/bin/env bash
# Build and run SI CDM.
#   ./run.sh            build (Release) and launch the app
#   ./run.sh debug      build (Debug) and launch the app
#   ./run.sh test       build and run the unit tests
#   ./run.sh xcode      generate the Xcode project and open it (macOS only)
#   ./run.sh clean      delete the build folder
set -euo pipefail

cd "$(dirname "$0")"

command -v cmake >/dev/null || { echo "cmake not found (macOS: brew install cmake)" >&2; exit 1; }

MODE="${1:-release}"
IS_MAC=false
[[ "$(uname)" == "Darwin" ]] && IS_MAC=true

case "$MODE" in
    clean)
        rm -rf build
        echo "Removed build/"
        exit 0
        ;;
    xcode)
        $IS_MAC || { echo "Xcode projects can only be generated on macOS." >&2; exit 1; }
        cmake --preset xcode
        open build/xcode/SICDM.xcodeproj
        echo "Opened build/xcode/SICDM.xcodeproj (scheme: sicdm)"
        exit 0
        ;;
    release | debug | test) ;;
    *)
        echo "Unknown option: $MODE  (use: release | debug | test | xcode | clean)" >&2
        exit 1
        ;;
esac

if $IS_MAC; then
    CONFIG=$([[ "$MODE" == "debug" ]] && echo Debug || echo Release)
    cmake --preset xcode
    cmake --build build/xcode --config "$CONFIG" -- -quiet
    if [[ "$MODE" == "test" ]]; then
        ctest --test-dir build/xcode -C "$CONFIG" --output-on-failure
    else
        open "build/xcode/$CONFIG/SICDM.app"
    fi
else
    PRESET=$([[ "$MODE" == "debug" ]] && echo debug || echo release)
    cmake --preset "$PRESET"
    cmake --build --preset "$PRESET" -j
    if [[ "$MODE" == "test" ]]; then
        ctest --test-dir "build/$PRESET" --output-on-failure
    else
        "./build/$PRESET/sicdm"
    fi
fi

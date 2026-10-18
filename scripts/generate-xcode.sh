#!/usr/bin/env bash
# Generates build/xcode/SICDM.xcodeproj and optionally opens it in Xcode.
#   ./scripts/generate-xcode.sh          generate + open
#   ./scripts/generate-xcode.sh --no-open
set -euo pipefail

cd "$(dirname "$0")/.."

if [[ "$(uname)" != "Darwin" ]]; then
    echo "Xcode projects can only be generated on macOS." >&2
    exit 1
fi
command -v cmake >/dev/null || { echo "cmake not found: brew install cmake" >&2; exit 1; }
xcode-select -p >/dev/null || { echo "Xcode command line tools missing: xcode-select --install" >&2; exit 1; }

cmake --preset xcode

if [[ "${1:-}" != "--no-open" ]]; then
    open build/xcode/SICDM.xcodeproj
fi
echo "Project: build/xcode/SICDM.xcodeproj  (scheme: sicdm)"

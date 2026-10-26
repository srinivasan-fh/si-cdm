#!/usr/bin/env bash
# UI smoke test: starts the app on each main screen (the landing page, the samples, the CAM
# panel and its windows, the user guide, preferences, licence, the 2D window, a drawing
# sheet, physics, the simulation window and the sculpted head), renders it and
# saves a screenshot. A crash, a failed command or a missing screenshot fails the run.
#
#   scripts/ui-smoke.sh [build/release/sicdm] [out-dir]
#
# On Linux without a display it runs under xvfb-run. The screenshots (PPM) are left in the
# out-dir (default: build/ui-smoke) to look through.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
app="${1:-$root/build/release/sicdm}"
out="${2:-$root/build/ui-smoke}"
[ -x "$app" ] || { echo "No app at $app (build it first)"; exit 2; }
mkdir -p "$out"
cd "$root"

# Keep the run away from the user's own preferences, recovery copies and crash reports.
state="$(mktemp -d)"
trap 'rm -rf "$state"' EXIT
export SICDM_PREFERENCES="$state/preferences.json"
export SICDM_RECOVERY_DIR="$state/recovery"
export SICDM_CRASH_DIR="$state/crashes"
export SICDM_LICENCE="$state/licence.json"

wrap=()
if [ "$(uname)" = Linux ] && [ -z "${DISPLAY:-}" ]; then
    command -v xvfb-run >/dev/null || { echo "Needs a display or xvfb-run"; exit 2; }
    wrap=(xvfb-run -a -s "-screen 0 1600x1000x24")
fi

failed=0
shot() { # name, then app arguments
    local name="$1"; shift
    local file="$out/$name.ppm" log="$out/$name.log"
    rm -f "$file"
    if "${wrap[@]}" "$app" --no-splash --size 1440 900 "$@" --screenshot "$file" >"$log" 2>&1 && [ -s "$file" ]; then
        printf '  ok    %s\n' "$name"
    else
        printf '  FAIL  %s (see %s)\n' "$name" "$log"
        failed=$((failed + 1))
    fi
}

echo "UI smoke test: $app"
shot landing
shot drawing-2d --2d
shot guide --command "PANEL GUIDE"
shot preferences --command "PREFERENCES SET units in" --command "PANEL PREFERENCES"
shot licence --command "PANEL LICENCE"
shot cad-sample --command "RUN docs/examples/cad-demo.txt" --command "PANEL MODEL"
shot library --command "PANEL LIBRARY"
shot cam-sample --command "RUN docs/examples/cam-demo.txt" --command "PANEL CAM"
shot cam-tools --command "RUN docs/examples/cam-demo.txt" --command "PANEL TOOLS"
shot cam-machine --command "RUN docs/examples/cam-demo.txt" --command "PANEL MACHINE"
shot cam-simulate --command "RUN docs/examples/cam-demo.txt" --command "CAM SIMULATE" --command "PANEL CAM"
shot lathe-sample --command "NEW" --command "RUN docs/examples/lathe-demo.txt" --command "PANEL CAM"
shot drawing-sheet --command "RUN docs/examples/cad-demo.txt" --command "SHEET NEW SIZE A3" --command "SHEET SHOW"
shot physics --command "PHYSICS DEMO" --command "SIM PLAY"
shot simulation --command "SIM"
shot robots --command "RUN docs/examples/robot-arm-sim.txt" --command "SIM"
shot sculpt-sample --command "RUN docs/examples/head-bust.txt"
shot datum-planes --command "NEW" --command "PREFERENCES SET units mm" --command "BOX 60 40 15 @0,0,0" --command "PLANE FACE 10,10,40 NAME Lid" --command "CIRCLE 0,0 6" --command "EXTRUDE 12" --command "PLANE THROUGH 0,0,15 30,0,40 0,20,15" --command "RECT 5,5 20,15" --command "EXTRUDE 4"
shot datum-2d --command "NEW" --command "PREFERENCES SET units mm" --command "BOX 60 40 15 @0,0,0" --command "PLANE ANGLE 30 ABOUT U FROM TOP NAME Tilt" --command "RECT 0,0 30,20" --command "CIRCLE 15,10 5" --command "2D Tilt"
shot extrude-options --command "NEW" --command "PREFERENCES SET units mm" --command "BOX 60 40 20 @0,0,0" --command "PLANE FACE 0,0,30 NAME Lid" --command "CIRCLE 0,0 5" --command "EXTRUDE THROUGH REVERSE CUT" --command "RECT -25,-15 -15,-5" --command "EXTRUDE 8 DRAFT 5 JOIN" --command "PLINE -20,10 20,10" --command "EXTRUDE 6 THIN 2 JOIN" --command "FIT"
shot save-reopen --command "BOX 20 20 20" --command "SAVE \"$state/smoke.sicdm\"" --command "NEW" --command "RECENT 1" --command "BOX 5 5 5" --command "UNDO"

if [ "$failed" -gt 0 ]; then
    echo "$failed screen(s) failed"
    exit 1
fi
echo "All screens rendered; screenshots in $out"

#pragma once

// Posted G-code read back the way the control would run it (a back-plot), and checked.
//
//   Program   every word is one the dialect knows; Fanuc-style controls get a decimal
//             point on lengths (X10 is 10 microns there); units, G54, a length offset
//             (G43, the loaded tool's H) before each tool moves in Z, the spindle on and a
//             feed before cutting, arcs whose ends are the same distance from the centre,
//             canned cycles with R above the bottom, G28 not driving down to Z0 first, and
//             an end (M30 / M2)
//   Machine   the travel (spindle nose, with each tool's length), rotary axes and their
//             limits, spindle speed, feed and the tool changer's pockets
//   Job       the back-plotted cutting path against the toolpath, both ways (nothing
//             cut that the toolpath doesn't cut, nothing in it missed), and no rapid move
//             across the stock below its top
//
// Arcs, helices and canned cycles (drill, dwell, peck, chip-break, tap, bore, and
// LinuxCNC's G33.1) are expanded as the control moves.

#include "core/Cam.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace sicdm {

struct GcodeIssue {
    int line = 0;       // 1-based line in the program (0: the whole program)
    bool error = true;  // false: a warning
    std::string text;
};

struct GcodeBackplot {
    struct Segment {
        double x0 = 0, y0 = 0, z0 = 0, x1 = 0, y1 = 0, z1 = 0; // work coordinates, mm
        double a = 0, c = 0; // the rotary axes at its end (degrees)
        bool rapid = false;
        int line = 0;
        int tool = 0;
    };
    std::vector<Segment> segments; // moves with known ends (not those to or from machine home)
    std::vector<GcodeIssue> issues;
    bool usesA = false, usesC = false;
    double aLo = 0, aHi = 0, cLo = 0, cHi = 0;
    double maxRpm = 0, maxFeed = 0; // feed: mm/min (G94 moves)
    double seconds = 0;
    std::vector<int> tools; // in the order they are loaded
    std::size_t lines = 0, arcs = 0, cycles = 0;
    std::size_t errors() const;
    std::size_t warnings() const;
};

// Runs the program for a dialect (fanuc, haas, grbl, linuxcnc) on a machine.
GcodeBackplot backplotGcode(const std::string& program, const std::string& dialect, const CamMachine& machine);

struct CamPostCheck {
    GcodeBackplot plot;
    std::vector<GcodeIssue> issues; // the program's, then the machine's and the job's
    double deviation = 0;           // farthest the cutting path strays from the toolpath, either way (mm)
    double allowed = 0;             // what the arc fitting and number rounding may account for
    std::size_t errors = 0, warnings = 0;
    std::string summary() const;    // one line
};

// Checks a program posted from `job` (by postToString) against the job and its machine.
CamPostCheck checkPost(const CamJob& job, const std::string& post, const std::string& program);

} // namespace sicdm

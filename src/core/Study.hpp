#pragma once

// Motion studies: the motions of a scene sampled over time (travel and angles, their
// velocity and acceleration, points on bodies), a contact follower's contact force on its
// spring, and where it would lift off; with a summary, a CSV table and an SVG chart.

#include "core/Math.hpp"

#include <string>
#include <vector>

namespace sicdm {

class Scene;

struct StudySeries {
    std::string name;  // "Follower travel"
    std::string unit;  // "mm"
    std::string group; // the chart it goes in: travel, angle, velocity, acceleration, force, position
    std::vector<double> v;
};

struct StudyRequest {
    double from = 0, to = 0; // seconds; to <= from: the scene's suggested length
    double step = 0;         // 0: 600 samples
    double smooth = 0;       // velocity and acceleration fitted over +- this (s); 0: chosen from the data
    std::vector<std::string> motions; // their names; empty (and no points): every enabled motion
    struct Point {
        std::string body;
        Vec3 at{};          // in the model, where the body is modelled
        bool given = false; // else the body's centre
    };
    std::vector<Point> points;
};

struct StudyResult {
    std::vector<double> t;
    std::vector<StudySeries> series;
    std::vector<std::string> notes; // peaks, lift-off and what would stop it
};

// Velocity and acceleration come from a local quadratic fit, over the fewest samples either
// side that settle it (a meshed cam's facets would otherwise show as spikes). A linear
// contact follower with a SPRING, PRELOAD or MASS also gets its contact force: mass x
// acceleration + preload + spring x lift above its lowest point (N); below zero it lifts off.
bool runStudy(const Scene& scene, const StudyRequest& request, StudyResult& out, std::string* why = nullptr);

// One line per series: its least and greatest value and when.
std::vector<std::string> studySummary(const StudyResult& r);
// t and every series, comma separated, with a header row.
std::string studyCsv(const StudyResult& r);
// The series charted against time, one panel per group, stacked.
std::string studySvg(const StudyResult& r, const std::string& title);

} // namespace sicdm

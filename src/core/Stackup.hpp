#pragma once

// Tolerance stack-up: a chain of dimensions (part sizes, drawing dimensions, gaps) adding up
// to a result, usually a gap or an overhang, with each one's tolerance. The result's range by
// worst case, by root sum of squares (RSS, statistical), by modified RSS (RSS x 1.5, Bender),
// and by a Monte Carlo run with each dimension's own distribution; against the spec limits,
// with what each dimension adds and what to tighten.
//
//   result = sum of  sign x sensitivity x dimension
//
// A vector stack (2D or 3D) is a loop of lengths, each along its own direction, and angles that
// turn the directions after them: the result is how far the loop's end is from its start, or
// that along a direction (MEASURE). It isn't a straight sum, so worst case and RSS use each
// dimension's sensitivity at the nominal sizes (worked out numerically), and the Monte Carlo
// run uses the loop itself.
//
// A dimension's tolerance is its band, upper and lower deviation from the nominal. A normal
// distribution fills the band at +-3 sigma x Cp (Cp 1: +-3 sigma); a uniform one evenly; a
// triangular one peaks at its middle. All lengths are mm.

#include "core/Math.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace sicdm {

class Json;
class Scene;
struct Drawing;

enum class StackDist { Normal, Uniform, Triangular };
const char* stackDistName(StackDist d);

struct StackItem {
    std::string name;
    double nominal = 0;            // mm
    double upper = 0, lower = 0;   // deviations from the nominal (mm): +0.1, -0.1
    int sign = 1;                  // +1 adds to the result, -1 takes from it
    double sens = 1;               // sensitivity: how much of it reaches the result (a lever, an angle's cosine)
    StackDist dist = StackDist::Normal;
    double cp = 1;                 // Normal: the band is +-3 sigma x cp
    // Where it came from, so UPDATE can measure it again:
    std::string body;              // a body's size ...
    Vec3 along{0, 0, 0};           // ... along this direction
    int dim = -1;                  // a drawing dimension (index)
    std::string fit;               // an ISO 286 fit code: the deviations follow the nominal
    char general = 0;              // an ISO 2768-1 class (f m c v): likewise
    // In a vector stack:
    Vec3 dir{1, 0, 0};             // a length's direction (turned by the angles before it)
    bool angle = false;            // an angle (degrees, its tolerance too), turning what follows ...
    Vec3 axis{0, 0, 1};            // ... about this
};

struct ToleranceStack {
    std::string name;
    std::string note;              // what the result is ("the lid's clearance")
    std::vector<StackItem> items;
    bool hasLow = false, hasHigh = false;
    double low = 0, high = 0;      // the spec limits on the result (mm)
    // Optional: the same result measured on the model, the gap from body `from` to body `to`
    // along `gapAlong` (to's near side less from's far side), to check the chain.
    std::string gapFrom, gapTo;
    Vec3 gapAlong{1, 0, 0};
    int samples = 100000;          // Monte Carlo samples
    // A vector stack (a loop in 2D or 3D): the result is the distance from the loop's start
    // to its end, or (when `measure` isn't zero) that along `measure`.
    bool vector = false;
    Vec3 measure{0, 0, 0};
};

struct StackResult {
    double nominal = 0;            // the result at every nominal
    double mean = 0;               // ... at every band's middle
    double wcLow = 0, wcHigh = 0;  // worst case
    double sigma = 0;              // statistical: sqrt(sum (sign sens sigma_i)^2)
    double skew = 0, kurtosis = 0; // a vector stack's result: its lean and (excess) tail weight (0 for a straight one)
    double rssLow = 0, rssHigh = 0;   // the 0.135% and 99.865% points: mean -+ 3 sigma (a vector stack: its own)
    double mrssLow = 0, mrssHigh = 0; // mean -+ 1.5 x 3 sigma, within the worst case
    std::vector<double> variance;  // each item's share of sigma^2 (%)
    std::vector<double> wcShare;   // ... of the worst-case band (%)
    // Against the spec (when it has limits):
    bool wcPass = true, rssPass = true;
    double ppm = 0;                // predicted outside the spec, from the RSS normal (parts per million)
    double cpk = 0;                // (nearest limit - mean) / 3 sigma; 0 without limits
    // Monte Carlo
    int samples = 0;
    double mcMean = 0, mcSigma = 0, mcLow = 0, mcHigh = 0; // mean, sigma, least and greatest seen
    double mcP0 = 0, mcP1 = 0;     // the 0.135% and 99.865% points (where +-3 sigma would be)
    double mcPpm = 0;              // the share of samples outside the spec (ppm)
    std::vector<int> histogram;    // 40 bins from histLow to histHigh
    double histLow = 0, histHigh = 0;
    // The model's own gap (when the stack names one).
    bool measured = false;
    double modelGap = 0;
    std::vector<std::string> notes; // findings and what to change
};

// The band of an ISO 2768-1 general tolerance (class f m c v) or ISO 286 fit code on a size.
bool stackItemFromFit(StackItem& item, std::string* why = nullptr);
bool stackItemFromGeneral(StackItem& item, std::string* why = nullptr);

// The result for these values of the items (in order): the sum, or the vector loop.
double stackValue(const ToleranceStack& stack, const std::vector<double>& values);
// How much the result moves for each item's change at the nominal sizes (a straight stack:
// sign x sensitivity; a vector one: worked out numerically).
std::vector<double> stackSensitivities(const ToleranceStack& stack);

// Each item's standard deviation (mm), before its sign and sensitivity.
double stackItemSigma(const StackItem& item);

// The analysis. `seed` makes the Monte Carlo run the same every time.
StackResult analyzeStack(const ToleranceStack& stack, std::uint64_t seed = 0x5eedc0de2768ull);

// A body's size along a direction (mm): its far side less its near side.
bool bodySizeAlong(const Scene& scene, const std::string& body, const Vec3& along, double& size, std::string* why = nullptr);
// The gap along a direction from one body to the next (its near side less the first's far side).
bool bodyGapAlong(const Scene& scene, const std::string& from, const std::string& to, const Vec3& along, double& gap, std::string* why = nullptr);
// Measures the items that come from the model or the drawing again, and the stack's gap.
// One line per change in `report`.
bool updateStack(ToleranceStack& stack, const Scene& scene, const Drawing& drawing, std::vector<std::string>* report, std::string* why = nullptr);
// The drawing dimension `index` as an item: its value and tolerance (the title block's
// general tolerance when it has none).
bool stackItemFromDrawing(StackItem& item, const Drawing& drawing, int index, std::string* why = nullptr);

// The analysis as text: the items, then worst case, RSS, Monte Carlo, the spec and notes.
std::vector<std::string> stackReport(const ToleranceStack& stack, const StackResult& r);
// The items and the results, comma separated.
std::string stackCsv(const ToleranceStack& stack, const StackResult& r);
// The Monte Carlo histogram with the spec and the worst-case and RSS limits, and the
// contributions as bars.
std::string stackSvg(const ToleranceStack& stack, const StackResult& r);

Json stacksToJson(const std::vector<ToleranceStack>& stacks);
std::vector<ToleranceStack> stacksFromJson(const Json& j);

} // namespace sicdm

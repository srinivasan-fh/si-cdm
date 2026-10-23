#include "core/Tolerances.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>

namespace sicdm {

namespace {

// ISO 286-1 size ranges: over the previous bound, up to and including this one (mm).
constexpr double kBounds[] = {3, 6, 10, 18, 30, 50, 80, 120, 180, 250, 315, 400, 500};
constexpr int kRanges = 13;

int rangeOf(double size) {
    if (!(size > 0)) return -1;
    for (int i = 0; i < kRanges; ++i)
        if (size <= kBounds[i] + 1e-9) return i;
    return -1;
}

// Standard tolerances IT5..IT11 (micrometres) per size range.
constexpr int kIT[7][kRanges] = {
    {4, 5, 6, 8, 9, 11, 13, 15, 18, 20, 23, 25, 27},              // IT5
    {6, 8, 9, 11, 13, 16, 19, 22, 25, 29, 32, 36, 40},            // IT6
    {10, 12, 15, 18, 21, 25, 30, 35, 40, 46, 52, 57, 63},         // IT7
    {14, 18, 22, 27, 33, 39, 46, 54, 63, 72, 81, 89, 97},         // IT8
    {25, 30, 36, 43, 52, 62, 74, 87, 100, 115, 130, 140, 155},    // IT9
    {40, 48, 58, 70, 84, 100, 120, 140, 160, 185, 210, 230, 250}, // IT10
    {60, 75, 90, 110, 130, 160, 190, 220, 250, 290, 320, 360, 400} // IT11
};

// Shaft fundamental deviations (micrometres): the upper deviation es for d e f g (and
// the holes D E F G mirror them as their lower deviation EI), the lower deviation ei for
// k (grades 4 to 7) m n p.
constexpr int kD[kRanges] = {-20, -30, -40, -50, -65, -80, -100, -120, -145, -170, -190, -210, -230};
constexpr int kE[kRanges] = {-14, -20, -25, -32, -40, -50, -60, -72, -85, -100, -110, -125, -135};
constexpr int kF[kRanges] = {-6, -10, -13, -16, -20, -25, -30, -36, -43, -50, -56, -62, -68};
constexpr int kG[kRanges] = {-2, -4, -5, -6, -7, -9, -10, -12, -14, -15, -17, -18, -20};
constexpr int kK[kRanges] = {0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 4, 5};
constexpr int kM[kRanges] = {2, 4, 6, 7, 8, 9, 11, 13, 15, 17, 20, 21, 23};
constexpr int kN[kRanges] = {4, 8, 10, 12, 15, 17, 20, 23, 27, 31, 34, 37, 40};
constexpr int kP[kRanges] = {6, 12, 15, 18, 22, 26, 32, 37, 43, 50, 56, 62, 68};

} // namespace

double itTolerance(int grade, double size) {
    const int r = rangeOf(size);
    if (r < 0 || grade < 5 || grade > 11) return 0;
    return kIT[grade - 5][r] / 1000.0;
}

bool isoFit(const std::string& code, double size, FitDeviations& out, std::string* why) {
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        return false;
    };
    // Letters, then the grade.
    std::size_t i = 0;
    while (i < code.size() && std::isalpha(static_cast<unsigned char>(code[i]))) ++i;
    const std::string letters = code.substr(0, i);
    if (letters.empty() || i >= code.size()) return fail("A fit is letters and a grade: H7, g6, js9");
    char* end = nullptr;
    const long grade = std::strtol(code.c_str() + i, &end, 10);
    if (!end || *end != '\0') return fail("A fit is letters and a grade: H7, g6, js9");
    if (grade < 5 || grade > 11) return fail("Grades 5 to 11 (IT5 to IT11)");
    const int r = rangeOf(size);
    if (r < 0) return fail("ISO 286 fits here cover sizes up to 500 mm");
    const double it = kIT[grade - 5][r] / 1000.0;
    const bool hole = std::isupper(static_cast<unsigned char>(letters[0]));
    std::string l;
    for (char c : letters) l += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (hole && letters.size() == 2 && !(std::isupper(static_cast<unsigned char>(letters[1])))) return fail("Write JS for a hole, js for a shaft");
    if (l == "js") {
        out = {it / 2, -it / 2};
        return true;
    }
    if (l.size() != 1) return fail("Fits: holes D E F G H JS, shafts d e f g h js k m n p");
    const char c = l[0];
    double es = 0; // for the letters given by their upper deviation (shafts d..h)
    switch (c) {
    case 'd': es = kD[r]; break;
    case 'e': es = kE[r]; break;
    case 'f': es = kF[r]; break;
    case 'g': es = kG[r]; break;
    case 'h': es = 0; break;
    case 'k':
    case 'm':
    case 'n':
    case 'p': {
        if (hole) return fail("Holes here: D E F G H JS (K M N P need ISO 286's corrections)");
        const double ei = (c == 'k' ? (grade <= 7 ? kK[r] : 0) : c == 'm' ? kM[r] : c == 'n' ? kN[r] : kP[r]) / 1000.0;
        out = {ei + it, ei};
        return true;
    }
    default: return fail("Fits: holes D E F G H JS, shafts d e f g h js k m n p");
    }
    es /= 1000.0;
    if (hole) out = {-es + it, -es}; // the hole's lower deviation mirrors the shaft's upper one
    else out = {es, es - it};
    return true;
}

double generalTolerance(char tolClass, double size) {
    // ISO 2768-1 linear sizes: over 0.5-3, 3-6, 6-30, 30-120, 120-400, 400-1000, 1000-2000, 2000-4000 mm.
    static const double bounds[] = {3, 6, 30, 120, 400, 1000, 2000, 4000};
    static const double f[] = {0.05, 0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 0};
    static const double m[] = {0.1, 0.1, 0.2, 0.3, 0.5, 0.8, 1.2, 2};
    static const double cc[] = {0.2, 0.3, 0.5, 0.8, 1.2, 2, 3, 4};
    static const double v[] = {0, 0.5, 1, 1.5, 2.5, 4, 6, 8};
    if (!(size >= 0.5)) return 0;
    const double* t = nullptr;
    switch (std::tolower(static_cast<unsigned char>(tolClass))) {
    case 'f': t = f; break;
    case 'm': t = m; break;
    case 'c': t = cc; break;
    case 'v': t = v; break;
    default: return 0;
    }
    for (int i = 0; i < 8; ++i)
        if (size <= bounds[i] + 1e-9) return t[i];
    return 0;
}

} // namespace sicdm

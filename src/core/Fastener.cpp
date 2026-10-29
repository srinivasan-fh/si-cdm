#include "core/Fastener.hpp"

#include "core/ExactMesh.hpp"
#include "core/HoleWizard.hpp"
#include "core/Scene.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace sicdm {

namespace {

constexpr float kIn = 25.4f;
constexpr int kSeg = 48; // around: a multiple of 6, so a hexagon's corners are ring points

std::string fmtG(float v) {
    char b[32];
    std::snprintf(b, sizeof b, "%g", static_cast<double>(v));
    return b;
}

std::string squash(const std::string& s) {
    std::string o;
    for (char c : s)
        if (!std::isspace(static_cast<unsigned char>(c))) o += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return o;
}

// A thread size's diameter and pitch from the hole wizard's table (the same sizes the holes use).
FastenerSize threaded(const char* name) {
    FastenerSize s;
    s.name = name;
    if (const HoleSize* h = findHoleSize(name)) s.d = h->nominal, s.pitch = h->pitch, s.inch = h->inch;
    return s;
}

// Metric rows: name, head (dia / across flats / OD), height, socket, socket depth, washer ID.
struct Row {
    const char* name;
    float head, height, socket, depth, hole;
};

std::vector<FastenerSize> metric(std::initializer_list<Row> rows) {
    std::vector<FastenerSize> out;
    for (const Row& r : rows) {
        FastenerSize s = threaded(r.name);
        s.head = r.head, s.height = r.height, s.socket = r.socket, s.depth = r.depth, s.hole = r.hole;
        out.push_back(s);
    }
    return out;
}

// Inch rows in inches; the socket depth, where not given, follows from the head.
void inch(std::vector<FastenerSize>& out, std::initializer_list<Row> rows) {
    for (const Row& r : rows) {
        FastenerSize s = threaded(r.name);
        s.head = r.head * kIn, s.height = r.height * kIn, s.socket = r.socket * kIn, s.depth = r.depth * kIn, s.hole = r.hole * kIn;
        out.push_back(s);
    }
}

std::vector<FastenerSize> pins(std::initializer_list<const char*> metricNames, std::initializer_list<const char*> inchNames) {
    std::vector<FastenerSize> out;
    for (const char* n : metricNames) {
        FastenerSize s;
        s.name = n, s.d = std::strtof(n, nullptr);
        out.push_back(s);
    }
    for (const char* n : inchNames) {
        FastenerSize s;
        s.name = n, s.inch = true;
        const std::string t = n;
        const std::size_t slash = t.find('/');
        s.d = (slash == std::string::npos ? std::strtof(n, nullptr) : std::strtof(n, nullptr) / std::strtof(n + slash + 1, nullptr)) * kIn;
        out.push_back(s);
    }
    return out;
}

const std::array<std::vector<FastenerSize>, static_cast<std::size_t>(FastenerType::Count)>& tables() {
    static const auto t = [] {
        std::array<std::vector<FastenerSize>, static_cast<std::size_t>(FastenerType::Count)> a;
        auto& hex = a[static_cast<std::size_t>(FastenerType::HexBolt)];
        // ISO 4017: across flats s, head height k.
        hex = metric({{"M1.6", 3.2f, 1.1f, 0, 0, 0}, {"M2", 4, 1.4f, 0, 0, 0},     {"M2.5", 5, 1.7f, 0, 0, 0},
                      {"M3", 5.5f, 2, 0, 0, 0},      {"M4", 7, 2.8f, 0, 0, 0},     {"M5", 8, 3.5f, 0, 0, 0},
                      {"M6", 10, 4, 0, 0, 0},        {"M8", 13, 5.3f, 0, 0, 0},    {"M10", 16, 6.4f, 0, 0, 0},
                      {"M12", 18, 7.5f, 0, 0, 0},    {"M14", 21, 8.8f, 0, 0, 0},   {"M16", 24, 10, 0, 0, 0},
                      {"M20", 30, 12.5f, 0, 0, 0},   {"M24", 36, 15, 0, 0, 0}});
        // ASME B18.2.1 hex cap screws: F, H.
        inch(hex, {{"1/4-20", 0.4375f, 0.156f, 0, 0, 0}, {"5/16-18", 0.5f, 0.203f, 0, 0, 0}, {"3/8-16", 0.5625f, 0.234f, 0, 0, 0},
                   {"1/2-13", 0.75f, 0.312f, 0, 0, 0},   {"5/8-11", 0.9375f, 0.391f, 0, 0, 0}, {"3/4-10", 1.125f, 0.469f, 0, 0, 0},
                   {"1-8", 1.5f, 0.609f, 0, 0, 0}});

        auto& shcs = a[static_cast<std::size_t>(FastenerType::SocketCap)];
        // ISO 4762: head dk, k, socket s, socket depth t.
        shcs = metric({{"M1.6", 3, 1.6f, 1.5f, 0.7f, 0}, {"M2", 3.8f, 2, 1.5f, 1, 0},   {"M2.5", 4.5f, 2.5f, 2, 1.1f, 0},
                       {"M3", 5.5f, 3, 2.5f, 1.3f, 0},   {"M4", 7, 4, 3, 2, 0},         {"M5", 8.5f, 5, 4, 2.5f, 0},
                       {"M6", 10, 6, 5, 3, 0},           {"M8", 13, 8, 6, 4, 0},        {"M10", 16, 10, 8, 5, 0},
                       {"M12", 18, 12, 10, 6, 0},        {"M14", 21, 14, 12, 7, 0},     {"M16", 24, 16, 14, 8, 0},
                       {"M20", 30, 20, 17, 10, 0},       {"M24", 36, 24, 19, 12, 0}});
        // ASME B18.3 socket head cap screws (1960 series): A, H, J; socket depth half the head.
        inch(shcs, {{"#2-56", 0.140f, 0.086f, 0.0781f, 0.043f, 0},  {"#4-40", 0.183f, 0.112f, 0.0938f, 0.056f, 0},
                    {"#6-32", 0.226f, 0.138f, 0.1094f, 0.069f, 0},  {"#8-32", 0.270f, 0.164f, 0.1406f, 0.082f, 0},
                    {"#10-24", 0.312f, 0.190f, 0.1562f, 0.095f, 0}, {"1/4-20", 0.375f, 0.250f, 0.1875f, 0.125f, 0},
                    {"5/16-18", 0.469f, 0.312f, 0.25f, 0.156f, 0},  {"3/8-16", 0.562f, 0.375f, 0.3125f, 0.188f, 0},
                    {"1/2-13", 0.750f, 0.500f, 0.375f, 0.250f, 0},  {"5/8-11", 0.938f, 0.625f, 0.5f, 0.312f, 0},
                    {"3/4-10", 1.125f, 0.750f, 0.625f, 0.375f, 0},  {"1-8", 1.5f, 1.0f, 0.75f, 0.5f, 0}});

        // Countersunk heads take the hole wizard's countersink diameter (the theoretical
        // sharp head, ISO 10642 / ASME B18.3 82 degree flat heads); the height follows from
        // the angle. Socket s.
        auto& csk = a[static_cast<std::size_t>(FastenerType::Countersunk)];
        auto countersunk = [&](const char* name, float socket) {
            FastenerSize s = threaded(name);
            const HoleSize* h = findHoleSize(name);
            s.head = h ? h->csinkDia : 0.0f;
            const float half = radians((h ? h->csinkAngle : 90.0f) * 0.5f);
            s.height = (s.head - s.d) * 0.5f / std::tan(half);
            s.socket = socket * (s.inch ? kIn : 1.0f);
            s.depth = 0.6f * s.height;
            csk.push_back(s);
        };
        for (auto [n, sk] : {std::pair{"M3", 2.0f}, {"M4", 2.5f}, {"M5", 3.0f}, {"M6", 4.0f}, {"M8", 5.0f}, {"M10", 6.0f},
                             {"M12", 8.0f}, {"M14", 10.0f}, {"M16", 10.0f}, {"M20", 12.0f}})
            countersunk(n, sk);
        for (auto [n, sk] : {std::pair{"#2-56", 0.050f}, {"#4-40", 0.0625f}, {"#6-32", 0.0781f}, {"#8-32", 0.0938f},
                             {"#10-24", 0.125f}, {"1/4-20", 0.1562f}, {"5/16-18", 0.1875f}, {"3/8-16", 0.2188f},
                             {"1/2-13", 0.3125f}, {"5/8-11", 0.375f}, {"3/4-10", 0.5f}, {"1-8", 0.625f}})
            countersunk(n, sk);

        auto& button = a[static_cast<std::size_t>(FastenerType::ButtonHead)];
        // ISO 7380-1: dk, k, s; socket depth 0.55 k.
        button = metric({{"M3", 5.7f, 1.65f, 2, 0, 0},  {"M4", 7.6f, 2.2f, 2.5f, 0, 0}, {"M5", 9.5f, 2.75f, 3, 0, 0},
                         {"M6", 10.5f, 3.3f, 4, 0, 0},  {"M8", 14, 4.4f, 5, 0, 0},      {"M10", 17.5f, 5.5f, 6, 0, 0},
                         {"M12", 21, 6.6f, 8, 0, 0},    {"M16", 28, 8.8f, 10, 0, 0}});
        // ASME B18.3 button heads: A, H, J.
        inch(button, {{"#4-40", 0.213f, 0.059f, 0.0625f, 0, 0},  {"#6-32", 0.262f, 0.073f, 0.0781f, 0, 0},
                      {"#8-32", 0.312f, 0.087f, 0.0938f, 0, 0},  {"#10-24", 0.361f, 0.101f, 0.125f, 0, 0},
                      {"1/4-20", 0.437f, 0.132f, 0.1562f, 0, 0}, {"5/16-18", 0.547f, 0.166f, 0.1875f, 0, 0},
                      {"3/8-16", 0.656f, 0.199f, 0.2188f, 0, 0}, {"1/2-13", 0.875f, 0.265f, 0.3125f, 0, 0},
                      {"5/8-11", 1.0f, 0.331f, 0.375f, 0, 0}});
        for (FastenerSize& s : button) s.depth = 0.55f * s.height;

        auto& set = a[static_cast<std::size_t>(FastenerType::SetScrew)];
        // ISO 4026 (flat point): socket s; ASME B18.3 set screws: J. Socket depth 0.5 d.
        set = metric({{"M1.6", 0, 0, 0.7f, 0, 0}, {"M2", 0, 0, 0.9f, 0, 0}, {"M2.5", 0, 0, 1.3f, 0, 0}, {"M3", 0, 0, 1.5f, 0, 0},
                      {"M4", 0, 0, 2, 0, 0},      {"M5", 0, 0, 2.5f, 0, 0}, {"M6", 0, 0, 3, 0, 0},      {"M8", 0, 0, 4, 0, 0},
                      {"M10", 0, 0, 5, 0, 0},     {"M12", 0, 0, 6, 0, 0},   {"M16", 0, 0, 8, 0, 0},     {"M20", 0, 0, 10, 0, 0},
                      {"M24", 0, 0, 12, 0, 0}});
        inch(set, {{"#2-56", 0, 0, 0.035f, 0, 0},  {"#4-40", 0, 0, 0.050f, 0, 0},   {"#6-32", 0, 0, 0.0625f, 0, 0},
                   {"#8-32", 0, 0, 0.0781f, 0, 0}, {"#10-24", 0, 0, 0.0938f, 0, 0}, {"1/4-20", 0, 0, 0.125f, 0, 0},
                   {"5/16-18", 0, 0, 0.1562f, 0, 0}, {"3/8-16", 0, 0, 0.1875f, 0, 0}, {"1/2-13", 0, 0, 0.25f, 0, 0},
                   {"5/8-11", 0, 0, 0.3125f, 0, 0}, {"3/4-10", 0, 0, 0.375f, 0, 0},  {"1-8", 0, 0, 0.5f, 0, 0}});
        for (FastenerSize& s : set) s.depth = 0.5f * s.d;

        auto& nut = a[static_cast<std::size_t>(FastenerType::HexNut)];
        // ISO 4032: s, m.
        nut = metric({{"M1.6", 3.2f, 1.3f, 0, 0, 0}, {"M2", 4, 1.6f, 0, 0, 0},    {"M2.5", 5, 2, 0, 0, 0},
                      {"M3", 5.5f, 2.4f, 0, 0, 0},   {"M4", 7, 3.2f, 0, 0, 0},    {"M5", 8, 4.7f, 0, 0, 0},
                      {"M6", 10, 5.2f, 0, 0, 0},     {"M8", 13, 6.8f, 0, 0, 0},   {"M10", 16, 8.4f, 0, 0, 0},
                      {"M12", 18, 10.8f, 0, 0, 0},   {"M14", 21, 12.8f, 0, 0, 0}, {"M16", 24, 14.8f, 0, 0, 0},
                      {"M20", 30, 18, 0, 0, 0},      {"M24", 36, 21.5f, 0, 0, 0}});
        // ASME B18.6.3 machine screw nuts (#2 to #10) and B18.2.2 finished hex nuts: F, H.
        inch(nut, {{"#2-56", 0.1875f, 0.0625f, 0, 0, 0}, {"#4-40", 0.25f, 0.094f, 0, 0, 0},     {"#6-32", 0.3125f, 0.109f, 0, 0, 0},
                   {"#8-32", 0.344f, 0.125f, 0, 0, 0},   {"#10-24", 0.375f, 0.125f, 0, 0, 0},   {"1/4-20", 0.4375f, 0.219f, 0, 0, 0},
                   {"5/16-18", 0.5f, 0.266f, 0, 0, 0},   {"3/8-16", 0.5625f, 0.328f, 0, 0, 0},  {"1/2-13", 0.75f, 0.438f, 0, 0, 0},
                   {"5/8-11", 0.9375f, 0.547f, 0, 0, 0}, {"3/4-10", 1.125f, 0.641f, 0, 0, 0},   {"1-8", 1.5f, 0.859f, 0, 0, 0}});

        auto& washer = a[static_cast<std::size_t>(FastenerType::Washer)];
        // ISO 7089: d2 (OD), h, d1 (ID).
        washer = metric({{"M1.6", 4, 0.3f, 0, 0, 1.7f},  {"M2", 5, 0.3f, 0, 0, 2.2f},    {"M2.5", 6, 0.5f, 0, 0, 2.7f},
                         {"M3", 7, 0.5f, 0, 0, 3.2f},    {"M4", 9, 0.8f, 0, 0, 4.3f},    {"M5", 10, 1, 0, 0, 5.3f},
                         {"M6", 12, 1.6f, 0, 0, 6.4f},   {"M8", 16, 1.6f, 0, 0, 8.4f},   {"M10", 20, 2, 0, 0, 10.5f},
                         {"M12", 24, 2.5f, 0, 0, 13},    {"M14", 28, 2.5f, 0, 0, 15},    {"M16", 30, 3, 0, 0, 17},
                         {"M20", 37, 3, 0, 0, 21},       {"M24", 44, 4, 0, 0, 25}});
        // ASME B18.22.1 type A plain washers, narrow (N): OD, thickness, ID.
        inch(washer, {{"#2-56", 0.250f, 0.020f, 0, 0, 0.094f}, {"#4-40", 0.312f, 0.032f, 0, 0, 0.125f},
                      {"#6-32", 0.375f, 0.049f, 0, 0, 0.156f}, {"#8-32", 0.438f, 0.049f, 0, 0, 0.188f},
                      {"#10-24", 0.5f, 0.049f, 0, 0, 0.219f},  {"1/4-20", 0.625f, 0.065f, 0, 0, 0.281f},
                      {"5/16-18", 0.688f, 0.065f, 0, 0, 0.344f}, {"3/8-16", 0.812f, 0.065f, 0, 0, 0.406f},
                      {"1/2-13", 1.062f, 0.095f, 0, 0, 0.531f}, {"5/8-11", 1.312f, 0.095f, 0, 0, 0.656f},
                      {"3/4-10", 1.469f, 0.134f, 0, 0, 0.812f}, {"1-8", 2.0f, 0.134f, 0, 0, 1.062f}});
        // A washer goes with a screw size: shown by the screw's name without its pitch.
        for (FastenerSize& s : washer) {
            const std::size_t dash = s.inch ? s.name.find('-', 1) : std::string::npos;
            if (dash != std::string::npos) s.name = s.name.substr(0, dash);
        }

        a[static_cast<std::size_t>(FastenerType::DowelPin)] =
            pins({"1", "1.5", "2", "2.5", "3", "4", "5", "6", "8", "10", "12", "16", "20"},
                 {"1/16", "3/32", "1/8", "5/32", "3/16", "1/4", "5/16", "3/8", "7/16", "1/2", "5/8", "3/4", "1"});
        return a;
    }();
    return t;
}

// ---------------------------------------------------------------------------
// Shape: rings stacked round the Z axis

struct Ring {
    bool hex = false; // a hexagon `size` across flats (else a circle of radius `size`)
    float size = 0;
    float z = 0;
    bool sphere = false; // the band from here to the next ring lies on the profile's sphere
};

struct Profile {
    std::vector<Ring> rings; // counter-clockwise in (radius, z): the outside first
    bool closed = false;     // the last ring joins the first (a nut, a washer)
    Vec3 sphereCentre{};
    float sphereRadius = 0;
};

Ring circle(float r, float z) { return {false, r, z, false}; }
Ring hexagon(float s, float z) { return {true, s, z, false}; }
float corner(float s) { return s * 0.5f / std::cos(kPi / 6.0f); } // a hexagon's corner radius
// How deep the drill point under a hex socket goes (118 degrees).
float apexDepth(float s) { return corner(s) / std::tan(radians(59.0f)); }

float ringRadius(const Ring& r, float a) {
    if (!r.hex) return r.size;
    const float sector = kPi / 3.0f;
    const float normal = kPi / 6.0f + sector * std::round((a - kPi / 6.0f) / sector); // the nearest flat's normal
    return r.size * 0.5f / std::cos(a - normal);
}

const FastenerSize* sizeOf(const std::vector<float>& p, FastenerType* type) {
    if (p.size() < 2) return nullptr;
    const int t = static_cast<int>(std::lround(p[0])), i = static_cast<int>(std::lround(p[1]));
    if (t < 0 || t >= static_cast<int>(FastenerType::Count)) return nullptr;
    const auto& list = tables()[static_cast<std::size_t>(t)];
    if (i < 0 || i >= static_cast<int>(list.size())) return nullptr;
    if (type) *type = static_cast<FastenerType>(t);
    return &list[static_cast<std::size_t>(i)];
}

bool profileOf(FastenerType type, const FastenerSize& z, float L, Profile& out, std::string* why) {
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        return false;
    };
    out = {};
    std::vector<Ring>& R = out.rings;
    const float d = z.d, r = d * 0.5f;
    if (!(d > 0)) return fail("Unknown size");
    const float c = z.pitch > 0 ? 0.8f * z.pitch : 0.1f * d; // the chamfer at the tip
    const float tol = 0.02f * d;
    const bool lengthy = fastenerHasLength(type);
    if (lengthy && !(L > 0 && L <= 2000.0f)) return fail("The length must be over 0");
    auto shank = [&] { // tip up to the bearing face
        R.push_back(circle(0, -L));
        R.push_back(circle(r - c, -L));
        R.push_back(circle(r, -L + c));
        R.push_back(circle(r, 0));
    };
    auto socket = [&](float top, float limitR) { // a hex socket down from z = top
        if (corner(z.socket) >= limitR - tol) return false;
        R.push_back(hexagon(z.socket, top));
        R.push_back(hexagon(z.socket, top - z.depth));
        R.push_back(circle(0, top - z.depth - apexDepth(z.socket)));
        return true;
    };
    switch (type) {
    case FastenerType::HexBolt:
        if (L < 1.5f * c + tol) return fail("Too short");
        shank();
        R.push_back(hexagon(z.head, 0));
        R.push_back(hexagon(z.head, z.height));
        R.push_back(circle(0, z.height));
        return true;
    case FastenerType::SocketCap: {
        if (L < 1.5f * c + tol) return fail("Too short");
        const float e = 0.06f * z.head; // the head's top edge, chamfered
        shank();
        R.push_back(circle(z.head * 0.5f, 0));
        R.push_back(circle(z.head * 0.5f, z.height - e));
        R.push_back(circle(z.head * 0.5f - e, z.height));
        if (z.height - z.depth - apexDepth(z.socket) < tol) return fail("The socket is too deep for the head");
        if (!socket(z.height, z.head * 0.5f - e)) return fail("The socket is too large for the head");
        return true;
    }
    case FastenerType::Countersunk: {
        const float kh = z.height; // the cone from the shank out to the head's top
        const float bottom = -z.depth - apexDepth(z.socket);
        if (L < kh + c + tol || bottom < -L + c + tol) return fail("Too short");
        shank();
        R.back().z = -kh; // the shank ends where the head's cone starts
        R.push_back(circle(z.head * 0.5f, 0));
        // The socket must stay inside the cone where it ends.
        const float coneR = z.depth < kh ? r + (z.head * 0.5f - r) * (kh - z.depth) / kh : r;
        if (!socket(0, coneR)) return fail("The socket is too large for the head");
        return true;
    }
    case FastenerType::ButtonHead: {
        if (L < 1.5f * c + tol) return fail("Too short");
        const float a = z.head * 0.5f, k = z.height, h1 = 0.2f * k;
        const float top = std::max(corner(z.socket) * 1.12f, 0.3f * z.head); // the flat round the socket
        // The dome: a sphere through (a, h1) and (top, k), centred on the axis.
        const float zc = (top * top + k * k - a * a - h1 * h1) / (2.0f * (k - h1));
        const float rs = std::sqrt(a * a + (h1 - zc) * (h1 - zc));
        const float bottom = k - z.depth - apexDepth(z.socket);
        if (bottom < -L + c + tol || (bottom < 0 && corner(z.socket) >= r - tol)) return fail("Too short");
        shank();
        R.push_back(circle(a, 0));
        R.push_back(circle(a, h1));
        R.back().sphere = true;
        constexpr int kDome = 6;
        for (int i = 1; i < kDome; ++i) {
            const float rho = a + (top - a) * static_cast<float>(i) / kDome;
            R.push_back(circle(rho, zc + std::sqrt(std::max(rs * rs - rho * rho, 0.0f))));
            R.back().sphere = true;
        }
        R.push_back(circle(top, k));
        out.sphereCentre = {0, 0, zc}, out.sphereRadius = rs;
        if (!socket(k, top)) return fail("The socket is too large for the head");
        return true;
    }
    case FastenerType::SetScrew: {
        const float rp = 0.65f * r, c2 = 0.1f * d; // the flat point, the top chamfer
        const float bottom = -z.depth - apexDepth(z.socket);
        if (bottom < -L + (r - rp) + tol) return fail("Too short");
        R.push_back(circle(0, -L));
        R.push_back(circle(rp, -L));
        R.push_back(circle(r, -L + (r - rp)));
        R.push_back(circle(r, -c2));
        R.push_back(circle(r - c2, 0));
        if (!socket(0, r - c2)) return fail("The socket is too large");
        return true;
    }
    case FastenerType::HexNut:
        R = {hexagon(z.head, -z.height), hexagon(z.head, 0), circle(r, 0), circle(r, -z.height)};
        out.closed = true;
        return true;
    case FastenerType::Washer:
        R = {circle(z.head * 0.5f, -z.height), circle(z.head * 0.5f, 0), circle(z.hole * 0.5f, 0), circle(z.hole * 0.5f, -z.height)};
        out.closed = true;
        return true;
    case FastenerType::DowelPin: {
        const float e = 0.1f * d;
        if (L < 3.0f * e + tol) return fail("Too short");
        R = {circle(0, -L * 0.5f), circle(r - e, -L * 0.5f), circle(r, -L * 0.5f + e), circle(r, L * 0.5f - e), circle(r - e, L * 0.5f),
             circle(0, L * 0.5f)};
        return true;
    }
    default: return fail("Unknown fastener");
    }
}

MeshData meshOf(const Profile& pr) {
    MeshData m;
    const std::size_t n = pr.rings.size();
    std::vector<std::array<Vec3, kSeg>> pts(n);
    for (std::size_t i = 0; i < n; ++i)
        for (int k = 0; k < kSeg; ++k) {
            const float a = 2.0f * kPi * static_cast<float>(k) / kSeg;
            const float rr = ringRadius(pr.rings[i], a);
            pts[i][static_cast<std::size_t>(k)] = rr == 0.0f ? Vec3{0, 0, pr.rings[i].z} : Vec3{rr * std::cos(a), rr * std::sin(a), pr.rings[i].z};
        }
    auto tri = [&](Vec3 a, Vec3 b, Vec3 c, const Vec3& hint) {
        Vec3 nn = cross(b - a, c - a);
        if (length(nn) < 1e-12f) return;
        nn = normalize(nn);
        if (dot(nn, hint) < 0.0f) {
            std::swap(b, c);
            nn = nn * -1.0f;
        }
        const auto base = static_cast<std::uint32_t>(m.vertices.size());
        m.vertices.insert(m.vertices.end(), {{a, nn}, {b, nn}, {c, nn}});
        m.indices.insert(m.indices.end(), {base, base + 1, base + 2});
    };
    const std::size_t bands = pr.closed ? n : n - 1;
    for (std::size_t i = 0; i < bands; ++i) {
        const auto& A = pts[i];
        const auto& B = pts[(i + 1) % n];
        const float dz = pr.rings[(i + 1) % n].z - pr.rings[i].z;
        for (int k = 0; k < kSeg; ++k) {
            const std::size_t k0 = static_cast<std::size_t>(k), k1 = static_cast<std::size_t>((k + 1) % kSeg);
            const float am = 2.0f * kPi * (static_cast<float>(k) + 0.5f) / kSeg;
            auto radial = [](const Vec3& p) { return std::sqrt(p.x * p.x + p.y * p.y); };
            const float dr = 0.5f * (radial(B[k0]) + radial(B[k1]) - radial(A[k0]) - radial(A[k1]));
            // Counter-clockwise in (radius, z): outward is (dz, -dr).
            const Vec3 hint{std::cos(am) * dz, std::sin(am) * dz, -dr};
            // Two triangles; a corner on the axis makes one of them vanish.
            tri(A[k0], A[k1], B[k1], hint);
            tri(A[k0], B[k1], B[k0], hint);
        }
    }
    m.computeFeatureEdges(25.0f);
    return m;
}

std::string lengthPart(float mm, bool inch) { return fastenerLengthText(mm, inch); }

} // namespace

const std::vector<FastenerSize>& fastenerSizes(FastenerType type) {
    static const std::vector<FastenerSize> none;
    const int t = static_cast<int>(type);
    return t >= 0 && t < static_cast<int>(FastenerType::Count) ? tables()[static_cast<std::size_t>(t)] : none;
}

const char* fastenerTypeName(FastenerType type) {
    switch (type) {
    case FastenerType::HexBolt: return "Hex bolt";
    case FastenerType::SocketCap: return "Socket head cap screw";
    case FastenerType::Countersunk: return "Countersunk screw";
    case FastenerType::ButtonHead: return "Button head screw";
    case FastenerType::SetScrew: return "Set screw";
    case FastenerType::HexNut: return "Hex nut";
    case FastenerType::Washer: return "Plain washer";
    case FastenerType::DowelPin: return "Dowel pin";
    default: return "Fastener";
    }
}

const char* fastenerKey(FastenerType type) {
    switch (type) {
    case FastenerType::HexBolt: return "hex_bolt";
    case FastenerType::SocketCap: return "socket_head_cap_screw";
    case FastenerType::Countersunk: return "countersunk_screw";
    case FastenerType::ButtonHead: return "button_head_screw";
    case FastenerType::SetScrew: return "set_screw";
    case FastenerType::HexNut: return "hex_nut";
    case FastenerType::Washer: return "washer";
    case FastenerType::DowelPin: return "dowel_pin";
    default: return "";
    }
}

const char* fastenerStandard(FastenerType type, const FastenerSize& s) {
    switch (type) {
    case FastenerType::HexBolt: return s.inch ? "ASME B18.2.1" : "ISO 4017";
    case FastenerType::SocketCap: return s.inch ? "ASME B18.3" : "ISO 4762";
    case FastenerType::Countersunk: return s.inch ? "ASME B18.3" : "ISO 10642";
    case FastenerType::ButtonHead: return s.inch ? "ASME B18.3" : "ISO 7380-1";
    case FastenerType::SetScrew: return s.inch ? "ASME B18.3" : "ISO 4026";
    case FastenerType::HexNut: return !s.inch ? "ISO 4032" : s.name[0] == '#' ? "ASME B18.6.3" : "ASME B18.2.2";
    case FastenerType::Washer: return s.inch ? "ASME B18.22.1" : "ISO 7089";
    case FastenerType::DowelPin: return s.inch ? "ASME B18.8.2" : "ISO 8734";
    default: return "";
    }
}

const char* fastenerChoices(FastenerType type) {
    static const auto lists = [] {
        std::array<std::string, static_cast<std::size_t>(FastenerType::Count)> a;
        for (std::size_t t = 0; t < a.size(); ++t)
            for (const FastenerSize& s : tables()[t]) a[t] += (a[t].empty() ? "" : "|") + s.name;
        return a;
    }();
    const int t = static_cast<int>(type);
    return t >= 0 && t < static_cast<int>(FastenerType::Count) ? lists[static_cast<std::size_t>(t)].c_str() : "";
}

int findFastenerSize(FastenerType type, const std::string& nameIn) {
    std::string n = squash(nameIn);
    if (n.empty()) return -1;
    const auto& list = fastenerSizes(type);
    if (type == FastenerType::DowelPin) {
        // "6", "6mm" metric; "1/4", "1/4in", "1in", 1/4" inch (a bare whole number is mm).
        bool inch = n.find('/') != std::string::npos;
        if (n.ends_with("mm")) n.resize(n.size() - 2), inch = false;
        else if (n.ends_with("in")) n.resize(n.size() - 2), inch = true;
        else if (n.back() == '"') n.pop_back(), inch = true;
        for (std::size_t i = 0; i < list.size(); ++i)
            if (list[i].inch == inch && squash(list[i].name) == n) return static_cast<int>(i);
        return -1;
    }
    // The hole wizard knows the spellings (M6x1, 10-24, #10-24); washers go by the screw.
    for (std::size_t i = 0; i < list.size(); ++i) {
        const std::string k = squash(list[i].name);
        if (n == k || (k[0] == '#' && n == k.substr(1))) return static_cast<int>(i);
    }
    if (const HoleSize* h = findHoleSize(nameIn)) {
        for (std::size_t i = 0; i < list.size(); ++i) {
            if (list[i].name == h->name) return static_cast<int>(i);
            const std::string full = h->name; // a washer's "1/4" for "1/4-20"
            if (type == FastenerType::Washer && list[i].inch && full.rfind(list[i].name + "-", 0) == 0) return static_cast<int>(i);
        }
    }
    return -1;
}

bool fastenerHasLength(FastenerType type) { return type != FastenerType::HexNut && type != FastenerType::Washer; }

float minFastenerLength(FastenerType type, int size) {
    const auto& list = fastenerSizes(type);
    if (!fastenerHasLength(type) || size < 0 || size >= static_cast<int>(list.size())) return 0.0f;
    const FastenerSize& z = list[static_cast<std::size_t>(size)];
    Profile pr;
    // The shortest sound shape (in steps of 0.1 mm), and no shorter than the diameter.
    for (float L = 0.1f; L < 60.0f * z.d; L += 0.1f)
        if (profileOf(type, z, L, pr, nullptr)) return std::max(std::ceil(L * 10.0f - 1e-3f), std::ceil(z.d * 10.0f - 1e-3f)) / 10.0f;
    return 0.0f;
}

std::vector<float> standardLengths(FastenerType type, bool inch) {
    std::vector<float> out;
    if (!fastenerHasLength(type)) return out;
    if (inch) {
        for (int i = 1; i <= 8; ++i) out.push_back(static_cast<float>(i) / 8.0f * kIn);       // 1/8 to 1
        for (int i = 5; i <= 12; ++i) out.push_back(static_cast<float>(i) / 4.0f * kIn);      // 1-1/4 to 3
        for (int i = 7; i <= 12; ++i) out.push_back(static_cast<float>(i) / 2.0f * kIn);      // 3-1/2 to 6
        for (int i = 7; i <= 12; ++i) out.push_back(static_cast<float>(i) * kIn);             // 7 to 12
        return out;
    }
    if (type == FastenerType::DowelPin)
        return {3, 4, 5, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 35, 40, 45, 50, 55, 60, 70, 80, 90, 100, 120, 140, 160, 180, 200};
    return {2, 2.5f, 3, 4, 5, 6, 8, 10, 12, 14, 16, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 90, 100,
            110, 120, 130, 140, 150, 160, 180, 200, 220, 240, 260, 280, 300};
}

float pickFastenerLength(FastenerType type, int size, float needed, bool atMost) {
    const auto& list = fastenerSizes(type);
    if (!fastenerHasLength(type) || size < 0 || size >= static_cast<int>(list.size())) return 0.0f;
    const float lo = minFastenerLength(type, size);
    std::vector<float> ls;
    for (float l : standardLengths(type, list[static_cast<std::size_t>(size)].inch))
        if (l >= lo - 1e-4f) ls.push_back(l);
    if (ls.empty()) return lo;
    if (atMost) {
        float best = 0;
        for (float l : ls)
            if (l <= needed + 1e-3f) best = l;
        return best > 0 ? best : ls.front();
    }
    for (float l : ls)
        if (l >= needed - 1e-3f) return l;
    // Past the table: on in steps of the last gap.
    const float step = ls.size() > 1 ? ls.back() - ls[ls.size() - 2] : 10.0f;
    return ls.back() + step * std::ceil((needed - ls.back()) / step);
}

MeshData buildFastener(const std::vector<float>& p, std::string* why) {
    FastenerType type{};
    const FastenerSize* z = sizeOf(p, &type);
    if (!z) {
        if (why) *why = "Unknown fastener type or size";
        return {};
    }
    Profile pr;
    if (!profileOf(type, *z, p.size() > 2 ? p[2] : 0.0f, pr, why)) return {};
    return meshOf(pr);
}

std::vector<BSurface> fastenerSurfaces(const std::vector<float>& p) {
    std::vector<BSurface> out;
    FastenerType type{};
    const FastenerSize* z = sizeOf(p, &type);
    Profile pr;
    if (!z || !profileOf(type, *z, p.size() > 2 ? p[2] : 0.0f, pr, nullptr)) return out;
    const std::size_t n = pr.rings.size();
    for (std::size_t i = 0; i < (pr.closed ? n : n - 1); ++i) {
        const Ring &a = pr.rings[i], &b = pr.rings[(i + 1) % n];
        if (a.sphere) continue;
        if (a.hex || b.hex) continue; // flat faces: found as planes
        for (const BSurface& s : revolveSurfaces({0, 0, 0}, {0, 0, 1}, {{a.size, a.z}, {b.size, b.z}})) out.push_back(s);
    }
    if (pr.sphereRadius > 0) {
        BSurface s;
        s.kind = BSurfaceKind::Sphere;
        s.f = Frame::make(DVec3(pr.sphereCentre), {0, 0, 1}, {1, 0, 0});
        s.r1 = pr.sphereRadius;
        out.push_back(s);
    }
    return out;
}

std::string fastenerLengthText(float mm, bool inch) {
    if (!inch) return fmtG(std::round(mm * 100.0f) / 100.0f);
    const double v = mm / kIn;
    const double sixtyFourths = std::round(v * 64.0);
    if (std::fabs(v * 64.0 - sixtyFourths) > 0.02) {
        char b[32];
        std::snprintf(b, sizeof b, "%.3f", v);
        return b;
    }
    int whole = static_cast<int>(sixtyFourths) / 64, num = static_cast<int>(sixtyFourths) % 64, den = 64;
    while (num > 0 && num % 2 == 0) num /= 2, den /= 2;
    if (num == 0) return std::to_string(whole);
    const std::string frac = std::to_string(num) + "/" + std::to_string(den);
    return whole > 0 ? std::to_string(whole) + "-" + frac : frac;
}

std::optional<float> parseFastenerLength(const std::string& text, bool inch) {
    const auto v = parseLength(text, inch ? LengthUnit::Inch : LengthUnit::Millimeter);
    if (!v || !(*v > 0)) return std::nullopt;
    return static_cast<float>(*v);
}

std::string fastenerDesignation(const std::vector<float>& p) {
    FastenerType type{};
    const FastenerSize* z = sizeOf(p, &type);
    if (!z) return {};
    std::string s = std::string(fastenerStandard(type, *z)) + " ";
    if (!fastenerHasLength(type)) return s + z->name;
    const std::string len = lengthPart(p.size() > 2 ? p[2] : 0.0f, z->inch);
    return s + z->name + (z->inch ? " x " + len : "x" + len);
}

std::string fastenerDescription(const std::vector<float>& p) {
    FastenerType type{};
    const FastenerSize* z = sizeOf(p, &type);
    if (!z) return {};
    std::string s = std::string(fastenerTypeName(type)) + " ";
    if (!fastenerHasLength(type)) return s + z->name;
    const std::string len = lengthPart(p.size() > 2 ? p[2] : 0.0f, z->inch);
    return s + z->name + (z->inch ? " x " + len : "x" + len);
}

std::string fastenerMaterial(FastenerType type, bool inch) {
    switch (type) {
    case FastenerType::HexBolt: return inch ? "Steel grade 5" : "Steel 8.8";
    case FastenerType::SocketCap: return inch ? "Alloy steel" : "Steel 12.9";
    case FastenerType::Countersunk:
    case FastenerType::ButtonHead: return inch ? "Alloy steel" : "Steel 10.9";
    case FastenerType::SetScrew: return inch ? "Alloy steel" : "Steel 45H";
    case FastenerType::HexNut: return inch ? "Steel grade 5" : "Steel 8";
    case FastenerType::Washer: return inch ? "Steel" : "Steel 200 HV";
    case FastenerType::DowelPin: return "Hardened steel";
    default: return "Steel";
    }
}

void applyFastenerProperties(Body& b) {
    if (b.kind != "fastener") return;
    FastenerType type{};
    const FastenerSize* z = sizeOf(b.params, &type);
    if (!z) return;
    b.properties["NUMBER"] = fastenerDesignation(b.params);
    b.properties["DESCRIPTION"] = fastenerDescription(b.params);
    if (!b.properties.count("MATERIAL")) b.properties["MATERIAL"] = fastenerMaterial(type, z->inch);
    if (z->pitch > 0) {
        if (z->inch) b.properties["THREAD"] = z->name + " UNC";
        else b.properties["THREAD"] = z->name + "x" + fmtG(z->pitch);
    } else {
        b.properties.erase("THREAD");
    }
}

Vec3 fastenerColour(FastenerType type) {
    const bool socket = type == FastenerType::SocketCap || type == FastenerType::Countersunk || type == FastenerType::ButtonHead ||
                        type == FastenerType::SetScrew;
    return socket ? Vec3{0.17f, 0.18f, 0.20f} : Vec3{0.78f, 0.80f, 0.84f};
}

Vec3 fastenerRotation(const Vec3& dir) {
    const float len = length(dir);
    if (!(len > 0)) return {};
    const Vec3 u = dir * (-1.0f / len); // where +Z goes
    const float ax = std::asin(std::clamp(-u.y, -1.0f, 1.0f));
    const float ay = std::atan2(u.x, u.z);
    return {ax * 180.0f / kPi + 0.0f, ay * 180.0f / kPi + 0.0f, 0.0f}; // no -0
}

} // namespace sicdm

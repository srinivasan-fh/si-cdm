#include "core/FeaReport.hpp"

#include "core/ImageWrite.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <functional>
#include <numeric>

namespace sicdm {

Vec3 feaFieldColour(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    const Vec3 a{0.10f, 0.25f, 0.95f}, b{0.10f, 0.85f, 0.95f}, c{0.20f, 0.90f, 0.25f}, d{0.98f, 0.90f, 0.15f}, e{0.95f, 0.15f, 0.10f};
    auto mix = [](Vec3 p, Vec3 q, float s) { return p + (q - p) * s; };
    if (t < 0.25f) return mix(a, b, t / 0.25f);
    if (t < 0.5f) return mix(b, c, (t - 0.25f) / 0.25f);
    if (t < 0.75f) return mix(c, d, (t - 0.5f) / 0.25f);
    return mix(d, e, (t - 0.75f) / 0.25f);
}

namespace {

std::string fmt(const char* f, double v) {
    char b[64];
    std::snprintf(b, sizeof b, f, v);
    return b;
}

std::string esc(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '<') o += "&lt;";
        else if (c == '>') o += "&gt;";
        else if (c == '&') o += "&amp;";
        else if (c == '"') o += "&quot;";
        else o += c;
    }
    return o;
}

std::string point(const Vec3& p) { return "(" + fmt("%.4g", p.x) + ", " + fmt("%.4g", p.y) + ", " + fmt("%.4g", p.z) + ")"; }

std::string where(const FeaSelection& s) {
    return s.kind == FeaSelection::Kind::Face ? "the face at " + point(s.point) : "within " + fmt("%.4g", s.radius) + " mm of " + point(s.point);
}

std::string hex(Vec3 c) {
    char b[16];
    std::snprintf(b, sizeof b, "#%02x%02x%02x", static_cast<int>(std::clamp(c.x, 0.0f, 1.0f) * 255 + 0.5f), static_cast<int>(std::clamp(c.y, 0.0f, 1.0f) * 255 + 0.5f),
                  static_cast<int>(std::clamp(c.z, 0.0f, 1.0f) * 255 + 0.5f));
    return b;
}

std::string base64(const std::string& in) {
    static const char* k = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string o;
    o.reserve((in.size() + 2) / 3 * 4);
    for (std::size_t i = 0; i < in.size(); i += 3) {
        const std::uint32_t v = (static_cast<std::uint32_t>(static_cast<unsigned char>(in[i])) << 16) |
                                (i + 1 < in.size() ? static_cast<std::uint32_t>(static_cast<unsigned char>(in[i + 1])) << 8 : 0u) |
                                (i + 2 < in.size() ? static_cast<std::uint32_t>(static_cast<unsigned char>(in[i + 2])) : 0u);
        o += k[(v >> 18) & 63], o += k[(v >> 12) & 63];
        o += i + 1 < in.size() ? k[(v >> 6) & 63] : '=';
        o += i + 2 < in.size() ? k[v & 63] : '=';
    }
    return o;
}

// The part seen from the isometric direction, coloured by its value (0..1) and shaded for
// its slope, its outline drawn; a PNG in the page, with the colour scale beside it.
std::string plotImage(const MeshData& mesh, const std::function<float(std::size_t)>& value, const std::function<Vec3(std::size_t)>& moved,
                      const std::string& lowText, const std::string& highText) {
    const Vec3 d = normalize(Vec3{-1, 1, -1});         // into the picture
    const Vec3 r = normalize(cross(d, Vec3{0, 0, 1})); // right
    const Vec3 u = cross(r, d);                        // up
    const Vec3 light = normalize(Vec3{-0.4f, 0.5f, 0.75f});
    constexpr int W = 600, H = 400, SS = 2, w = W * SS, h = H * SS; // drawn twice the size, then averaged down
    std::vector<Vec3> p(mesh.vertices.size());
    float x0 = 1e30f, x1 = -1e30f, y0 = 1e30f, y1 = -1e30f;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec3 q = moved(i);
        p[i] = {dot(q, r), dot(q, u), dot(q, d)};
        x0 = std::min(x0, p[i].x), x1 = std::max(x1, p[i].x), y0 = std::min(y0, p[i].y), y1 = std::max(y1, p[i].y);
    }
    const float pad = 14.0f * SS;
    const float sc = std::min((static_cast<float>(w) - 2 * pad) / std::max(x1 - x0, 1e-6f), (static_cast<float>(h) - 2 * pad) / std::max(y1 - y0, 1e-6f));
    const float ox = (static_cast<float>(w) - (x1 - x0) * sc) / 2, oy = (static_cast<float>(h) - (y1 - y0) * sc) / 2;
    for (Vec3& q : p) q = {ox + (q.x - x0) * sc, static_cast<float>(h) - (oy + (q.y - y0) * sc), q.z};
    std::vector<float> zb(static_cast<std::size_t>(w) * h, 1e30f);
    std::vector<Vec3> col(static_cast<std::size_t>(w) * h, Vec3{1, 1, 1});
    for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const std::uint32_t i0 = mesh.indices[t], i1 = mesh.indices[t + 1], i2 = mesh.indices[t + 2];
        const Vec3 &a = p[i0], &b = p[i1], &c = p[i2];
        const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        if (std::fabs(area) < 1e-9f) continue;
        const Vec3 n = normalize(cross(moved(i1) - moved(i0), moved(i2) - moved(i0)));
        const float shade = 0.55f + 0.45f * std::fabs(dot(n, light));
        const float v0 = value(i0), v1 = value(i1), v2 = value(i2);
        const int xa = std::max(0, static_cast<int>(std::floor(std::min({a.x, b.x, c.x})))), xb = std::min(w - 1, static_cast<int>(std::ceil(std::max({a.x, b.x, c.x}))));
        const int ya = std::max(0, static_cast<int>(std::floor(std::min({a.y, b.y, c.y})))), yb = std::min(h - 1, static_cast<int>(std::ceil(std::max({a.y, b.y, c.y}))));
        for (int y = ya; y <= yb; ++y)
            for (int x = xa; x <= xb; ++x) {
                const float px = static_cast<float>(x) + 0.5f, py = static_cast<float>(y) + 0.5f;
                const float w0 = ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) / area;
                const float w1 = ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) / area;
                const float w2 = 1 - w0 - w1;
                if (w0 < 0 || w1 < 0 || w2 < 0) continue;
                const float z = w0 * a.z + w1 * b.z + w2 * c.z;
                const std::size_t k = static_cast<std::size_t>(y) * w + static_cast<std::size_t>(x);
                if (z >= zb[k]) continue;
                zb[k] = z;
                col[k] = feaFieldColour(w0 * v0 + w1 * v1 + w2 * v2) * shade;
            }
    }
    // The outline: where the depth jumps (the part's edge against the paper, or a step).
    const float jump = 0.02f * std::max(x1 - x0, y1 - y0); // model mm
    std::vector<Vec3> ink = col;
    for (int y = 1; y + 1 < h; ++y)
        for (int x = 1; x + 1 < w; ++x) {
            const std::size_t k = static_cast<std::size_t>(y) * w + static_cast<std::size_t>(x);
            const float z = zb[k];
            const bool paper = z >= 1e29f;
            for (const std::size_t o : {k - 1, k + 1, k - static_cast<std::size_t>(w), k + static_cast<std::size_t>(w)}) {
                const bool op = zb[o] >= 1e29f;
                if (paper != op || (!paper && std::fabs(zb[o] - z) > jump)) {
                    if (!paper) ink[k] = Vec3{0.15f, 0.15f, 0.18f};
                    break;
                }
            }
        }
    std::vector<std::uint8_t> px(static_cast<std::size_t>(W) * H * 3);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            Vec3 sum{};
            for (int j = 0; j < SS; ++j)
                for (int i = 0; i < SS; ++i) sum = sum + ink[static_cast<std::size_t>(y * SS + j) * w + static_cast<std::size_t>(x * SS + i)];
            sum = sum * (1.0f / (SS * SS));
            const std::size_t k = (static_cast<std::size_t>(y) * W + static_cast<std::size_t>(x)) * 3;
            px[k] = static_cast<std::uint8_t>(std::clamp(sum.x, 0.0f, 1.0f) * 255 + 0.5f);
            px[k + 1] = static_cast<std::uint8_t>(std::clamp(sum.y, 0.0f, 1.0f) * 255 + 0.5f);
            px[k + 2] = static_cast<std::uint8_t>(std::clamp(sum.z, 0.0f, 1.0f) * 255 + 0.5f);
        }
    std::string o = "<div class=\"plot\"><img alt=\"result plot\" width=\"600\" height=\"400\" src=\"data:image/png;base64," +
                    base64(encodePngCompressed(W, H, 3, px)) + "\">";
    o += "<div class=\"scale\"><span>" + esc(highText) + "</span><div class=\"bar\" style=\"background:linear-gradient(to top";
    for (int k = 0; k <= 4; ++k) o += "," + hex(feaFieldColour(static_cast<float>(k) / 4.0f));
    o += ")\"></div><span>" + esc(lowText) + "</span></div></div>";
    return o;
}

} // namespace

std::string feaReportHtml(const FeaReportData& rd) {
    const FeaSetup& S = rd.setup;
    const FeaResult& R = rd.result;
    const bool statics = !S.loads.empty() || !S.pressures.empty() || !S.bearings.empty() || length(S.gravity) > 0 || (S.thermalStress && !S.temperatures.empty());
    const bool heat = !S.temperatures.empty();
    std::string date;
    {
        const std::time_t now = std::time(nullptr);
        char b[32];
        std::tm tmv{};
#ifdef _WIN32
        localtime_s(&tmv, &now);
#else
        localtime_r(&now, &tmv);
#endif
        std::strftime(b, sizeof b, "%Y-%m-%d %H:%M", &tmv);
        date = b;
    }
    std::string h = "<!doctype html>\n<html><head><meta charset=\"utf-8\"><title>Analysis of " + esc(rd.body) + "</title>\n<style>\n"
                    "body{font-family:sans-serif;margin:24px auto;max-width:900px;color:#222;background:#fff}\n"
                    "h1{font-size:22px;margin-bottom:2px}h2{font-size:16px;margin-top:26px;border-bottom:1px solid #ccc;padding-bottom:3px}\n"
                    "table{border-collapse:collapse;margin:6px 0}td,th{border:1px solid #ccc;padding:4px 10px;text-align:left;font-size:13px}\n"
                    "th{background:#f2f2f2}.ok{color:#17702a;font-weight:bold}.warn{color:#9a6a00;font-weight:bold}.bad{color:#b0201a;font-weight:bold}\n"
                    ".sub{color:#666;font-size:13px}figure{margin:10px 0}figcaption{font-size:13px;color:#444}\n"
                    ".plot{display:flex;align-items:stretch;gap:10px}.plot img{border:1px solid #ddd;max-width:100%;height:auto}\n"
                    ".scale{display:flex;flex-direction:column;justify-content:space-between;font-size:12px;padding:30px 0}\n"
                    ".scale .bar{flex:1;width:18px;border:1px solid #888;margin:4px 0}\n"
                    "@media print{h2{page-break-after:avoid}figure{page-break-inside:avoid}}\n</style></head><body>\n";
    h += "<h1>Analysis of " + esc(rd.body) + "</h1>\n<div class=\"sub\">" + (rd.title.empty() ? std::string() : esc(rd.title) + " &middot; ") + esc(date) +
         " &middot; linear static, natural frequencies and steady heat by 8-node cubes</div>\n";
    // The verdict first.
    h += "<h2>Summary</h2>\n<table>\n";
    if (statics) {
        const std::string cls = R.safetyFactor < 1 ? "bad" : R.safetyFactor < 1.5 ? "warn" : "ok";
        const std::string say = R.safetyFactor < 1 ? "yields: the stress passes the yield strength" : R.safetyFactor < 1.5 ? "thin margin" : "holds";
        h += "<tr><th>Factor of safety</th><td class=\"" + cls + "\">" + fmt("%.3g", R.safetyFactor) + " (" + say + ")</td></tr>\n";
        h += "<tr><th>Max von Mises stress</th><td>" + fmt("%.4g", R.maxStress) + " MPa at " + point(R.maxStressAt) + "</td></tr>\n";
        h += "<tr><th>Max displacement</th><td>" + fmt("%.4g", R.maxDisplacement) + " mm at " + point(R.maxDisplacementAt) + "</td></tr>\n";
    }
    if (!R.frequencies.empty()) h += "<tr><th>First natural frequency</th><td>" + fmt("%.5g", R.frequencies[0]) + " Hz</td></tr>\n";
    if (heat)
        h += "<tr><th>Temperatures</th><td>" + fmt("%.4g", R.minTemperature) + " &deg;C to " + fmt("%.4g", R.maxTemperature) + " &deg;C</td></tr>\n";
    h += "<tr><th>Mass</th><td>" + fmt("%.4g", R.mass) + " kg</td></tr>\n";
    h += std::string("<tr><th>Solver</th><td>") + (R.converged ? "converged" : "<span class=\"warn\">stopped before converging: rough results</span>") + " in " +
         std::to_string(R.iterations) + " steps</td></tr>\n</table>\n";
    // What went in.
    const FeaMaterial& M = S.material;
    h += "<h2>Material</h2>\n<table><tr><th>Name</th><td>" + esc(M.name) + "</td></tr><tr><th>Young's modulus</th><td>" + fmt("%g", M.E) +
         " MPa</td></tr><tr><th>Poisson's ratio</th><td>" + fmt("%g", M.nu) + "</td></tr><tr><th>Yield strength</th><td>" + fmt("%g", M.yield) +
         " MPa</td></tr><tr><th>Density</th><td>" + fmt("%g", M.density) + " g/cm&sup3;</td></tr>";
    if (heat) h += "<tr><th>Conductivity</th><td>" + fmt("%g", M.conductivity) + " W/(m K)</td></tr><tr><th>Expansion</th><td>" + fmt("%g", M.expansion) + " &times;10&#8315;&#8310;/K</td></tr>";
    h += "</table>\n";
    h += "<h2>Model</h2>\n<table><tr><th>Cubes</th><td>" + std::to_string(R.cells) + " of " + fmt("%.4g", R.cell) + " mm</td></tr><tr><th>Nodes</th><td>" +
         std::to_string(R.nodes) + "</td></tr><tr><th>Fixed nodes</th><td>" + std::to_string(R.fixedNodes) + "</td></tr><tr><th>Loaded nodes</th><td>" +
         std::to_string(R.loadedNodes) + "</td></tr></table>\n";
    h += "<h2>Fixings and loads</h2>\n<table><tr><th>Kind</th><th>Where</th><th>Value</th></tr>\n";
    for (const FeaSelection& f : S.fixed) h += "<tr><td>Fixed</td><td>" + where(f) + "</td><td>held still</td></tr>\n";
    for (const FeaLoad& l : S.loads) h += "<tr><td>Force</td><td>" + where(l.where) + "</td><td>" + point(l.force) + " N (" + fmt("%.4g", length(l.force)) + " N)</td></tr>\n";
    for (const FeaPressure& p : S.pressures) h += "<tr><td>Pressure</td><td>" + where(p.where) + "</td><td>" + fmt("%.4g", p.mpa) + " MPa</td></tr>\n";
    for (const FeaBearing& b : S.bearings)
        h += "<tr><td>Bearing load</td><td>the hole at " + point(b.center) + ", radius " + fmt("%.4g", b.radius) + " mm</td><td>" + point(b.force) + " N</td></tr>\n";
    if (length(S.gravity) > 0) h += "<tr><td>Own weight</td><td>the whole body</td><td>" + point(S.gravity) + " g</td></tr>\n";
    for (const FeaTemperature& t : S.temperatures) h += "<tr><td>Temperature</td><td>" + where(t.where) + "</td><td>" + fmt("%.4g", t.celsius) + " &deg;C</td></tr>\n";
    for (const FeaHeat& q : S.heats) h += "<tr><td>Heat flow</td><td>" + where(q.where) + "</td><td>" + fmt("%.4g", q.watts) + " W</td></tr>\n";
    if (S.thermalStress) h += "<tr><td>Thermal expansion</td><td>the whole body</td><td>free of stress at " + fmt("%.4g", S.reference) + " &deg;C</td></tr>\n";
    h += "</table>\n";
    // What came out.
    if (!R.frequencies.empty()) {
        h += "<h2>Natural frequencies</h2>\n<table><tr><th>Mode</th><th>Frequency</th></tr>\n";
        for (std::size_t m = 0; m < R.frequencies.size(); ++m) h += "<tr><td>" + std::to_string(m + 1) + "</td><td>" + fmt("%.5g", R.frequencies[m]) + " Hz</td></tr>\n";
        h += "</table>\n";
    }
    const MeshData& mesh = rd.mesh;
    const std::size_t nv = mesh.vertices.size();
    auto still = [&](std::size_t i) { return mesh.vertices[i].position; };
    if (!mesh.indices.empty()) {
        h += "<h2>Results on the part</h2>\n";
        if (statics && R.surfaceStress.size() == nv) {
            const double top = R.maxStress > 0 ? R.maxStress : 1.0;
            h += "<figure>" +
                 plotImage(mesh, [&](std::size_t i) { return static_cast<float>(R.surfaceStress[i] / top); }, still, "0 MPa", fmt("%.4g MPa", R.maxStress)) +
                 "<figcaption>von Mises stress, 0 (blue) to " + fmt("%.4g", R.maxStress) + " MPa (red)</figcaption></figure>\n";
        }
        if (statics && R.surfaceDisplacement.size() == nv) {
            const double top = R.maxDisplacement > 0 ? R.maxDisplacement : 1.0;
            // The shape drawn moved, scaled so the largest movement is a twentieth of the part.
            float size = 0;
            const AABB b = mesh.bounds();
            if (b.valid()) size = std::max({b.max.x - b.min.x, b.max.y - b.min.y, b.max.z - b.min.z});
            const float k = R.maxDisplacement > 0 ? static_cast<float>(0.05 * size / R.maxDisplacement) : 0.0f;
            h += "<figure>" +
                 plotImage(
                     mesh, [&](std::size_t i) { return static_cast<float>(length(R.surfaceDisplacement[i]) / top); },
                     [&](std::size_t i) { return mesh.vertices[i].position + R.surfaceDisplacement[i] * k; }, "0 mm", fmt("%.4g mm", R.maxDisplacement)) +
                 "<figcaption>Displacement, 0 (blue) to " + fmt("%.4g", R.maxDisplacement) + " mm (red); the shape drawn " + fmt("%.3g", k) +
                 " times its true movement</figcaption></figure>\n";
        }
        if (heat && R.surfaceTemperature.size() == nv) {
            const double lo = R.minTemperature, span = std::max(1e-9, R.maxTemperature - R.minTemperature);
            h += "<figure>" +
                 plotImage(mesh, [&](std::size_t i) { return static_cast<float>((R.surfaceTemperature[i] - lo) / span); }, still, fmt("%.4g \xC2\xB0" "C", lo),
                         fmt("%.4g \xC2\xB0" "C", R.maxTemperature)) +
                 "<figcaption>Temperature, " + fmt("%.4g", lo) + " &deg;C (blue) to " + fmt("%.4g", R.maxTemperature) + " &deg;C (red)</figcaption></figure>\n";
        }
        for (std::size_t m = 0; m < R.modeShapes.size() && m < 3; ++m) {
            const std::vector<Vec3>& shape = R.modeShapes[m];
            if (shape.size() != nv) continue;
            float size = 0;
            const AABB b = mesh.bounds();
            if (b.valid()) size = std::max({b.max.x - b.min.x, b.max.y - b.min.y, b.max.z - b.min.z});
            h += "<figure>" +
                 plotImage(
                     mesh, [&](std::size_t i) { return std::min(1.0f, length(shape[i])); }, [&](std::size_t i) { return mesh.vertices[i].position + shape[i] * (0.08f * size); },
                     "still", "most") +
                 "<figcaption>Mode " + std::to_string(m + 1) + " at " + fmt("%.5g", R.frequencies[m]) + " Hz: motion, none (blue) to most (red)</figcaption></figure>\n";
        }
    }
    h += "<h2>Notes</h2>\n<p class=\"sub\">Linear elastic, small displacements, isotropic material. The body is filled with cubes on a regular grid; "
         "stresses are found at the cubes' centres, so sharp inside corners read lower than a fine mesh would show and sharp re-entrant corners have no "
         "finite stress at all. Halving the cube size (CELL) and seeing the results settle is the check that they are converged.</p>\n";
    h += "</body></html>\n";
    return h;
}

} // namespace sicdm

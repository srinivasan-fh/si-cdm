#include "core/Terrain.hpp"

#include "core/Json.hpp"

#include <stb_image.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>

namespace sicdm {

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr float kBase = -50.0f; // the slab's underside (mm)

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

const struct {
    TerrainKind k;
    const char* key;
} kKinds[] = {{TerrainKind::None, "off"},     {TerrainKind::Slope, "slope"},   {TerrainKind::Stairs, "stairs"},
              {TerrainKind::Rough, "rough"},  {TerrainKind::Blocks, "blocks"}, {TerrainKind::Dem, "dem"}};

// The patch's own frame: u along `direction`, v across it, from the centre.
void local(const TerrainSettings& t, double x, double y, double& u, double& v) {
    const double c = std::cos(t.direction * kDeg), s = std::sin(t.direction * kDeg);
    const double dx = x - t.centre.x, dy = y - t.centre.y;
    u = dx * c + dy * s;
    v = -dx * s + dy * c;
}

Vec3 world(const TerrainSettings& t, double u, double v, double z) {
    const double c = std::cos(t.direction * kDeg), s = std::sin(t.direction * kDeg);
    return {static_cast<float>(t.centre.x + u * c - v * s), static_cast<float>(t.centre.y + u * s + v * c), static_cast<float>(z)};
}

std::uint32_t hash(std::int64_t x, std::int64_t y, int seed) {
    std::uint64_t h = static_cast<std::uint64_t>(x) * 0x9E3779B97F4A7C15ull ^ static_cast<std::uint64_t>(y) * 0xC2B2AE3D27D4EB4Full ^
                      static_cast<std::uint64_t>(seed) * 0x165667B19E3779F9ull;
    h ^= h >> 31;
    h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 29;
    return static_cast<std::uint32_t>(h);
}
double unit(std::uint32_t h) { return (h & 0xFFFFFF) / static_cast<double>(0xFFFFFF); }

// Smooth value noise in [0, 1].
double valueNoise(double x, double y, int seed) {
    const double fx = std::floor(x), fy = std::floor(y);
    const std::int64_t ix = static_cast<std::int64_t>(fx), iy = static_cast<std::int64_t>(fy);
    double tx = x - fx, ty = y - fy;
    tx = tx * tx * (3 - 2 * tx);
    ty = ty * ty * (3 - 2 * ty);
    const double a = unit(hash(ix, iy, seed)), b = unit(hash(ix + 1, iy, seed)), c = unit(hash(ix, iy + 1, seed)), d = unit(hash(ix + 1, iy + 1, seed));
    return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
}

// Fractal noise: three octaves, in [0, 1].
double fractal(double x, double y, int seed) {
    return (valueNoise(x, y, seed) * 4 + valueNoise(x * 2.1, y * 2.1, seed + 7) * 2 + valueNoise(x * 4.3, y * 4.3, seed + 13)) / 7.0;
}

double gridHeight(const TerrainSettings& t, double u, double v) {
    if (t.nx < 2 || t.ny < 2 || t.cell <= 0) return 0;
    const double gx = (u + t.sizeX * 0.5) / t.cell, gy = (v + t.sizeY * 0.5) / t.cell;
    const double cx = std::clamp(gx, 0.0, t.nx - 1.000001), cy = std::clamp(gy, 0.0, t.ny - 1.000001);
    const int i = static_cast<int>(cx), j = static_cast<int>(cy);
    const double fx = cx - i, fy = cy - j;
    auto at = [&](int a, int b) { return static_cast<double>(t.grid[static_cast<std::size_t>(b) * static_cast<std::size_t>(t.nx) + static_cast<std::size_t>(a)]); };
    return (at(i, j) * (1 - fx) + at(i + 1, j) * fx) * (1 - fy) + (at(i, j + 1) * (1 - fx) + at(i + 1, j + 1) * fx) * fy;
}

// Height in the patch frame (inside the patch).
double localHeight(const TerrainSettings& t, double u, double v) {
    switch (t.kind) {
    case TerrainKind::None: return 0;
    case TerrainKind::Slope: return std::max(0.0, (u + t.sizeX * 0.5) * std::tan(t.angle * kDeg));
    case TerrainKind::Stairs: {
        if (u < 0 || t.run <= 0) return 0;
        const double k = std::floor(u / t.run);
        return std::min(k + 1.0, static_cast<double>(std::max(t.steps, 0))) * t.rise;
    }
    case TerrainKind::Rough: {
        // Fades to the floor over the last 200 mm of the patch, so its edge is no cliff.
        const double edge = std::min({u + t.sizeX * 0.5, t.sizeX * 0.5 - u, v + t.sizeY * 0.5, t.sizeY * 0.5 - v});
        const double fade = std::clamp(edge / 200.0, 0.0, 1.0);
        const double wl = std::max(t.wavelength, 1.0f);
        return fade * t.amplitude * fractal(u / wl, v / wl, t.seed);
    }
    case TerrainKind::Blocks: {
        const double s = std::max(t.blockSize, 1.0f);
        const std::int64_t i = static_cast<std::int64_t>(std::floor(u / s)), j = static_cast<std::int64_t>(std::floor(v / s));
        const std::uint32_t h = hash(i, j, t.seed);
        return (h & 1) ? t.blockHeight * (0.3 + 0.7 * unit(h >> 1)) : 0.0;
    }
    case TerrainKind::Dem: return gridHeight(t, u, v);
    }
    return 0;
}

bool inside(const TerrainSettings& t, double u, double v) { return std::fabs(u) <= t.sizeX * 0.5 && std::fabs(v) <= t.sizeY * 0.5; }

void addTri(MeshData& m, const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 n = normalize(cross(b - a, c - a));
    const std::uint32_t base = static_cast<std::uint32_t>(m.vertices.size());
    for (const Vec3& p : {a, b, c}) m.vertices.push_back({p, n, 0.0f});
    m.indices.insert(m.indices.end(), {base, base + 1, base + 2});
}
void addQuad(MeshData& m, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
    addTri(m, a, b, c);
    addTri(m, a, c, d);
}

// A box in the patch frame.
void addBox(MeshData& m, const TerrainSettings& t, double u0, double u1, double v0, double v1, double z0, double z1) {
    const Vec3 p[8] = {world(t, u0, v0, z0), world(t, u1, v0, z0), world(t, u1, v1, z0), world(t, u0, v1, z0),
                       world(t, u0, v0, z1), world(t, u1, v0, z1), world(t, u1, v1, z1), world(t, u0, v1, z1)};
    addQuad(m, p[0], p[3], p[2], p[1]);
    addQuad(m, p[4], p[5], p[6], p[7]);
    addQuad(m, p[0], p[1], p[5], p[4]);
    addQuad(m, p[2], p[3], p[7], p[6]);
    addQuad(m, p[1], p[2], p[6], p[5]);
    addQuad(m, p[3], p[0], p[4], p[7]);
}

} // namespace

const char* terrainKindKey(TerrainKind k) noexcept {
    for (const auto& e : kKinds)
        if (e.k == k) return e.key;
    return "off";
}

std::optional<TerrainKind> terrainKindFromKey(const std::string& key) noexcept {
    const std::string k = lower(key);
    for (const auto& e : kKinds)
        if (k == e.key) return e.k;
    if (k == "none" || k == "flat") return TerrainKind::None;
    if (k == "ramp" || k == "incline") return TerrainKind::Slope;
    if (k == "stair" || k == "steps") return TerrainKind::Stairs;
    if (k == "uneven" || k == "noise") return TerrainKind::Rough;
    if (k == "heightmap" || k == "elevation") return TerrainKind::Dem;
    return std::nullopt;
}

Json terrainToJson(const TerrainSettings& t) {
    Json j = Json::object();
    j.set("kind", terrainKindKey(t.kind));
    j.set("size", Json(Json::Array{Json(t.sizeX), Json(t.sizeY)}));
    j.set("centre", Json(Json::Array{Json(t.centre.x), Json(t.centre.y)}));
    j.set("direction", t.direction);
    j.set("angle", t.angle);
    j.set("rise", t.rise), j.set("run", t.run), j.set("steps", t.steps);
    j.set("amplitude", t.amplitude), j.set("wavelength", t.wavelength);
    j.set("block_height", t.blockHeight), j.set("block_size", t.blockSize);
    j.set("seed", t.seed);
    j.set("friction", t.friction);
    if (t.kind == TerrainKind::Dem) {
        j.set("dem_path", t.demPath);
        j.set("nx", t.nx), j.set("ny", t.ny), j.set("cell", t.cell);
        Json g = Json::array();
        for (float h : t.grid) g.push(std::round(h * 10.0f) / 10.0f);
        j.set("grid", g);
    }
    return j;
}

bool terrainFromJson(const Json& j, TerrainSettings& t) {
    t = {};
    const auto k = terrainKindFromKey(j["kind"].asString(std::string("off")));
    if (!k) return false;
    t.kind = *k;
    t.sizeX = j["size"][0].asFloat(4000), t.sizeY = j["size"][1].asFloat(4000);
    t.centre = {j["centre"][0].asFloat(0), j["centre"][1].asFloat(0)};
    t.direction = j["direction"].asFloat(0), t.angle = j["angle"].asFloat(10);
    t.rise = j["rise"].asFloat(150), t.run = j["run"].asFloat(300), t.steps = j["steps"].asInt(6);
    t.amplitude = j["amplitude"].asFloat(40), t.wavelength = j["wavelength"].asFloat(400);
    t.blockHeight = j["block_height"].asFloat(60), t.blockSize = j["block_size"].asFloat(250);
    t.seed = j["seed"].asInt(1), t.friction = j["friction"].asFloat(0.9f);
    if (t.kind == TerrainKind::Dem) {
        t.demPath = j["dem_path"].asString(std::string());
        t.nx = j["nx"].asInt(0), t.ny = j["ny"].asInt(0), t.cell = j["cell"].asFloat(0);
        const Json& g = j["grid"];
        for (std::size_t i = 0; i < g.size(); ++i) t.grid.push_back(g[i].asFloat(0));
        if (t.nx < 2 || t.ny < 2 || t.grid.size() != static_cast<std::size_t>(t.nx) * static_cast<std::size_t>(t.ny)) t.kind = TerrainKind::None;
    }
    return true;
}

double terrainHeight(const TerrainSettings& t, double x, double y) {
    if (!t.active()) return 0;
    double u, v;
    local(t, x, y, u, v);
    return inside(t, u, v) ? localHeight(t, u, v) : 0.0;
}

Vec3 terrainNormal(const TerrainSettings& t, double x, double y) {
    if (!t.active()) return {0, 0, 1};
    // Stairs and blocks are flat on their treads: the normal of the tread, not of the riser.
    if (t.kind == TerrainKind::Stairs || t.kind == TerrainKind::Blocks) return {0, 0, 1};
    const double h = 2.0;
    const double dx = (terrainHeight(t, x + h, y) - terrainHeight(t, x - h, y)) / (2 * h);
    const double dy = (terrainHeight(t, x, y + h) - terrainHeight(t, x, y - h)) / (2 * h);
    return normalize(Vec3{static_cast<float>(-dx), static_cast<float>(-dy), 1.0f});
}

MeshData terrainMesh(const TerrainSettings& t) {
    MeshData m;
    if (!t.active()) return m;
    const double hx = t.sizeX * 0.5, hy = t.sizeY * 0.5;
    if (t.kind == TerrainKind::Stairs || t.kind == TerrainKind::Blocks) {
        // Exact boxes: the base slab, then each step (or block) on it.
        addBox(m, t, -hx, hx, -hy, hy, kBase, 0);
        if (t.kind == TerrainKind::Stairs) {
            for (int k = 0; k < t.steps && t.run > 0; ++k) {
                const double u0 = static_cast<double>(k) * t.run;
                const double u1 = k + 1 == t.steps ? hx : std::min(hx, static_cast<double>(k + 1) * t.run);
                if (u0 >= hx) break;
                addBox(m, t, u0, u1, -hy, hy, 0, static_cast<double>(k + 1) * t.rise);
            }
        } else {
            const double s = std::max(t.blockSize, 1.0f);
            for (double u0 = std::floor(-hx / s) * s; u0 < hx; u0 += s)
                for (double v0 = std::floor(-hy / s) * s; v0 < hy; v0 += s) {
                    const double h = localHeight(t, u0 + s * 0.5, v0 + s * 0.5);
                    if (h <= 0) continue;
                    addBox(m, t, std::max(u0, -hx), std::min(u0 + s, hx), std::max(v0, -hy), std::min(v0 + s, hy), 0, h);
                }
        }
        m.computeFeatureEdges(30.0f);
        return m;
    }
    // A height field: the top on a grid of at most 256 x 256, the sides and the bottom.
    const int nx = std::clamp(static_cast<int>(t.sizeX / 20.0f), 8, 256), ny = std::clamp(static_cast<int>(t.sizeY / 20.0f), 8, 256);
    auto top = [&](int i, int j) {
        const double u = -hx + static_cast<double>(t.sizeX) * i / nx, v = -hy + static_cast<double>(t.sizeY) * j / ny;
        return world(t, u, v, localHeight(t, u, v));
    };
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i) addQuad(m, top(i, j), top(i + 1, j), top(i + 1, j + 1), top(i, j + 1));
    auto low = [&](const Vec3& p) { return Vec3{p.x, p.y, kBase}; };
    for (int i = 0; i < nx; ++i) {
        addQuad(m, low(top(i, 0)), low(top(i + 1, 0)), top(i + 1, 0), top(i, 0));
        addQuad(m, low(top(i + 1, ny)), low(top(i, ny)), top(i, ny), top(i + 1, ny));
    }
    for (int j = 0; j < ny; ++j) {
        addQuad(m, low(top(0, j + 1)), low(top(0, j)), top(0, j), top(0, j + 1));
        addQuad(m, low(top(nx, j)), low(top(nx, j + 1)), top(nx, j + 1), top(nx, j));
    }
    addQuad(m, low(top(0, 0)), low(top(0, ny)), low(top(nx, ny)), low(top(nx, 0)));
    m.computeFeatureEdges(60.0f);
    return m;
}

bool loadDem(const std::string& path, TerrainSettings& t, float cellMm, float heightMm, std::string* error) {
    auto fail = [&](const std::string& why) {
        if (error) *error = why;
        return false;
    };
    std::string ext;
    if (const auto dot = path.find_last_of('.'); dot != std::string::npos) ext = lower(path.substr(dot));
    int w = 0, h = 0;
    std::vector<float> heights; // row 0 = the top (+Y), mm
    float cell = cellMm;
    if (ext == ".asc") {
        std::ifstream in(path);
        if (!in) return fail("Can't read " + path);
        std::string key;
        double nodata = -9999, cellsize = 0;
        // Header: key value lines until the numbers start.
        for (int k = 0; k < 6 && in >> key;) {
            const std::string kk = lower(key);
            double v = 0;
            if (kk == "ncols") in >> w;
            else if (kk == "nrows") in >> h;
            else if (kk == "cellsize") in >> cellsize;
            else if (kk == "nodata_value") in >> nodata;
            else if (kk == "xllcorner" || kk == "yllcorner" || kk == "xllcenter" || kk == "yllcenter") in >> v;
            else {
                // A number: the header is over.
                std::istringstream back(key);
                double first = 0;
                if (!(back >> first)) return fail(path + " is not an ESRI ASCII grid");
                heights.push_back(static_cast<float>(first));
                break;
            }
            ++k;
        }
        if (w < 2 || h < 2) return fail(path + ": the grid needs ncols and nrows of 2 or more");
        // 64-bit counts (long is 32 bits on Windows), and no more cells than a grid can sensibly have.
        const std::int64_t cells = static_cast<std::int64_t>(w) * static_cast<std::int64_t>(h);
        if (cells > 100'000'000) return fail(path + ": ncols x nrows is too large");
        for (double v; static_cast<std::int64_t>(heights.size()) < cells && in >> v;) heights.push_back(static_cast<float>(v));
        if (static_cast<std::int64_t>(heights.size()) < cells) return fail(path + ": fewer heights than ncols x nrows");
        float lo = 1e30f;
        for (float& v : heights)
            if (v != static_cast<float>(nodata)) lo = std::min(lo, v);
        for (float& v : heights) v = v == static_cast<float>(nodata) ? 0.0f : (v - lo) * 1000.0f;
        // Heights the engines can work with: finite, and under 100 km of relief.
        for (float v : heights)
            if (!std::isfinite(v) || v > 1e8f) return fail(path + ": heights out of range (more than 100 km of relief, or not numbers)");
        if (cell <= 0) cell = static_cast<float>(cellsize * 1000.0);
    } else {
        int n = 0;
        unsigned char* px = stbi_load(path.c_str(), &w, &h, &n, 1);
        if (!px) return fail("Can't read the image " + path);
        const float range = heightMm > 0 ? heightMm : 1000.0f;
        heights.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
        for (std::size_t i = 0; i < heights.size(); ++i) heights[i] = px[i] / 255.0f * range;
        stbi_image_free(px);
        if (w < 2 || h < 2) return fail(path + " is too small");
    }
    if (cell <= 0) cell = 50.0f;
    // Resample to at most 400 x 400 points (row 0 of the result at -Y).
    const int sx = std::max(1, (std::max(w, h) + 399) / 400);
    t.nx = (w - 1) / sx + 1;
    t.ny = (h - 1) / sx + 1;
    t.cell = cell * static_cast<float>(sx);
    t.grid.assign(static_cast<std::size_t>(t.nx) * static_cast<std::size_t>(t.ny), 0.0f);
    for (int j = 0; j < t.ny; ++j)
        for (int i = 0; i < t.nx; ++i) {
            const int srcRow = h - 1 - j * sx; // the file's top row is +Y
            t.grid[static_cast<std::size_t>(j) * static_cast<std::size_t>(t.nx) + static_cast<std::size_t>(i)] =
                heights[static_cast<std::size_t>(std::max(srcRow, 0)) * static_cast<std::size_t>(w) + static_cast<std::size_t>(i * sx)];
        }
    t.kind = TerrainKind::Dem;
    t.demPath = path;
    t.sizeX = t.cell * static_cast<float>(t.nx - 1);
    t.sizeY = t.cell * static_cast<float>(t.ny - 1);
    return true;
}

} // namespace sicdm

#include "core/Fea.hpp"

#include "core/Armature.hpp"
#include "core/Wearable.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <functional>
#include <thread>
#include <unordered_map>

namespace sicdm {

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

// The stiffness of a unit cube (E = 1) of 8-node trilinear elements, 2 x 2 x 2 Gauss points.
// Nodes in the order (x, y, z) = (i & 1, (i >> 1) & 1, (i >> 2) & 1).
std::array<double, 24 * 24> unitStiffness(double nu) {
    std::array<double, 24 * 24> K{};
    const double lam = nu / ((1 + nu) * (1 - 2 * nu)), mu = 1 / (2 * (1 + nu));
    double D[6][6] = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) D[i][j] = lam + (i == j ? 2 * mu : 0);
    D[3][3] = D[4][4] = D[5][5] = mu;
    const double g = 1 / std::sqrt(3.0);
    for (int gp = 0; gp < 8; ++gp) {
        const double xi[3] = {0.5 + 0.5 * g * ((gp & 1) ? 1 : -1), 0.5 + 0.5 * g * ((gp & 2) ? 1 : -1), 0.5 + 0.5 * g * ((gp & 4) ? 1 : -1)};
        double dN[8][3];
        for (int n = 0; n < 8; ++n) {
            const double s[3] = {static_cast<double>(n & 1), static_cast<double>((n >> 1) & 1), static_cast<double>((n >> 2) & 1)};
            double f[3];
            for (int k = 0; k < 3; ++k) f[k] = s[k] > 0.5 ? xi[k] : 1 - xi[k];
            for (int k = 0; k < 3; ++k) {
                double d = s[k] > 0.5 ? 1 : -1;
                for (int m = 0; m < 3; ++m)
                    if (m != k) d *= f[m];
                dN[n][k] = d;
            }
        }
        double B[6][24] = {};
        for (int n = 0; n < 8; ++n) {
            B[0][3 * n] = dN[n][0], B[1][3 * n + 1] = dN[n][1], B[2][3 * n + 2] = dN[n][2];
            B[3][3 * n] = dN[n][1], B[3][3 * n + 1] = dN[n][0];
            B[4][3 * n + 1] = dN[n][2], B[4][3 * n + 2] = dN[n][1];
            B[5][3 * n] = dN[n][2], B[5][3 * n + 2] = dN[n][0];
        }
        double DB[6][24];
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 24; ++j) {
                double v = 0;
                for (int k = 0; k < 6; ++k) v += D[i][k] * B[k][j];
                DB[i][j] = v;
            }
        for (int i = 0; i < 24; ++i)
            for (int j = 0; j < 24; ++j) {
                double v = 0;
                for (int k = 0; k < 6; ++k) v += B[k][i] * DB[k][j];
                K[static_cast<std::size_t>(i * 24 + j)] += v / 8.0; // weight: the unit cube's volume over 8 points
            }
    }
    return K;
}

} // namespace

const std::vector<FeaMaterial>& feaMaterials() {
    static const std::vector<FeaMaterial> k = {
        {"steel", 200000, 0.30, 250, 7.85, 50, 12},     {"stainless", 193000, 0.29, 215, 8.0, 16, 17}, {"aluminium", 69000, 0.33, 275, 2.70, 167, 23},
        {"titanium", 114000, 0.34, 880, 4.43, 7, 8.6},  {"brass", 100000, 0.34, 200, 8.5, 109, 19},    {"copper", 117000, 0.34, 70, 8.96, 390, 17},
        {"abs", 2100, 0.35, 40, 1.04, 0.17, 90},        {"pla", 3500, 0.36, 50, 1.24, 0.13, 68},        {"petg", 2100, 0.38, 50, 1.27, 0.2, 60},
        {"nylon", 2700, 0.39, 50, 1.14, 0.25, 80},
    };
    return k;
}

std::optional<FeaMaterial> feaMaterial(const std::string& name) {
    std::string n = lowerOf(name);
    if (n == "aluminum" || n == "al" || n == "6061") n = "aluminium";
    if (n == "ss" || n == "304") n = "stainless";
    for (const FeaMaterial& m : feaMaterials())
        if (m.name == n) return m;
    return std::nullopt;
}

namespace {

// The conduction of a unit cube (k = 1): the integral of grad N_a . grad N_b, 2 x 2 x 2 points.
std::array<double, 64> unitConduction() {
    std::array<double, 64> K{};
    const double g = 1 / std::sqrt(3.0);
    for (int gp = 0; gp < 8; ++gp) {
        const double xi[3] = {0.5 + 0.5 * g * ((gp & 1) ? 1 : -1), 0.5 + 0.5 * g * ((gp & 2) ? 1 : -1), 0.5 + 0.5 * g * ((gp & 4) ? 1 : -1)};
        double dN[8][3];
        for (int n = 0; n < 8; ++n) {
            const double sg[3] = {static_cast<double>(n & 1), static_cast<double>((n >> 1) & 1), static_cast<double>((n >> 2) & 1)};
            double f[3];
            for (int k = 0; k < 3; ++k) f[k] = sg[k] > 0.5 ? xi[k] : 1 - xi[k];
            for (int k = 0; k < 3; ++k) {
                double d = sg[k] > 0.5 ? 1 : -1;
                for (int m = 0; m < 3; ++m)
                    if (m != k) d *= f[m];
                dN[n][k] = d;
            }
        }
        for (int a = 0; a < 8; ++a)
            for (int b = 0; b < 8; ++b) K[static_cast<std::size_t>(a * 8 + b)] += (dN[a][0] * dN[b][0] + dN[a][1] * dN[b][1] + dN[a][2] * dN[b][2]) / 8.0;
    }
    return K;
}

// The body filled with cubes: their corner nodes, which nodes are on the surface.
struct Grid {
    float h = 0;
    Vec3 origin;
    int nx = 0, ny = 0, nz = 0;
    std::vector<char> solid;
    std::unordered_map<long long, int> nodeOf;
    std::vector<std::array<int, 8>> elems;
    std::vector<std::array<int, 3>> elemIdx;
    std::vector<Vec3> nodePos;
    std::vector<char> surface;
    bool cellAt(int i, int j, int k) const {
        if (i < 0 || j < 0 || k < 0 || i >= nx || j >= ny || k >= nz) return false;
        return solid[(static_cast<std::size_t>(k) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(j)) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)] != 0;
    }
    int nodeAt(long long a, long long b, long long c) const {
        const auto it = nodeOf.find((c * (ny + 1) + b) * (nx + 1) + a);
        return it == nodeOf.end() ? -1 : it->second;
    }
};

const int kFaces[6][4] = {{0, 2, 4, 6}, {1, 3, 5, 7}, {0, 1, 4, 5}, {2, 3, 6, 7}, {0, 1, 2, 3}, {4, 5, 6, 7}};
const int kStep[6][3] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};

bool buildGrid(const MeshData& world, float cell, int maxCells, Grid& g, std::string& why) {
    const AABB box = world.bounds();
    const Vec3 size = box.max - box.min;
    const float longest = std::max({size.x, size.y, size.z});
    if (!(longest > 0)) return why = "The body has no size", false;
    float h = cell > 0 ? cell : longest / 40.0f;
    int count = 0;
    for (int attempt = 0; attempt < 12; ++attempt) {
        // Cube faces on the bounding box's: a block's faces fall on cube faces.
        g.origin = box.min;
        g.nx = std::max(1, static_cast<int>(std::ceil(size.x / h - 1e-4f))), g.ny = std::max(1, static_cast<int>(std::ceil(size.y / h - 1e-4f))),
        g.nz = std::max(1, static_cast<int>(std::ceil(size.z / h - 1e-4f)));
        const long long total = static_cast<long long>(g.nx) * g.ny * g.nz;
        if (total > 40LL * maxCells) { // too fine to even try
            h *= 1.25f;
            continue;
        }
        g.solid.assign(static_cast<std::size_t>(total), 0);
        // Along each row of cube centres (in x), the stretches inside the surface.
        std::vector<std::vector<float>> hits(static_cast<std::size_t>(g.ny) * static_cast<std::size_t>(g.nz));
        for (std::size_t t = 0; t + 2 < world.indices.size(); t += 3) {
            const Vec3 a = world.vertices[world.indices[t]].position, b = world.vertices[world.indices[t + 1]].position,
                       c = world.vertices[world.indices[t + 2]].position;
            const float y0 = std::min({a.y, b.y, c.y}), y1 = std::max({a.y, b.y, c.y}), z0 = std::min({a.z, b.z, c.z}), z1 = std::max({a.z, b.z, c.z});
            const int j0 = std::max(0, static_cast<int>(std::ceil((y0 - g.origin.y) / h - 0.5f))), j1 = std::min(g.ny - 1, static_cast<int>(std::floor((y1 - g.origin.y) / h - 0.5f)));
            const int k0 = std::max(0, static_cast<int>(std::ceil((z0 - g.origin.z) / h - 0.5f))), k1 = std::min(g.nz - 1, static_cast<int>(std::floor((z1 - g.origin.z) / h - 0.5f)));
            for (int k = k0; k <= k1; ++k)
                for (int j = j0; j <= j1; ++j) {
                    // A tiny shift keeps rays off edges and vertices.
                    const double py = g.origin.y + (j + 0.5) * h + 1.3e-5 * h, pz = g.origin.z + (k + 0.5) * h + 0.7e-5 * h;
                    const double d1 = (b.y - a.y) * (pz - a.z) - (b.z - a.z) * (py - a.y);
                    const double d2 = (c.y - b.y) * (pz - b.z) - (c.z - b.z) * (py - b.y);
                    const double d3 = (a.y - c.y) * (pz - c.z) - (a.z - c.z) * (py - c.y);
                    if (!((d1 >= 0 && d2 >= 0 && d3 >= 0) || (d1 <= 0 && d2 <= 0 && d3 <= 0))) continue;
                    const Vec3 n = cross(b - a, c - a);
                    if (std::fabs(n.x) < 1e-20f) continue;
                    const double x = a.x - (n.y * (py - a.y) + n.z * (pz - a.z)) / n.x;
                    hits[static_cast<std::size_t>(k) * static_cast<std::size_t>(g.ny) + static_cast<std::size_t>(j)].push_back(static_cast<float>(x));
                }
        }
        count = 0;
        for (int k = 0; k < g.nz; ++k)
            for (int j = 0; j < g.ny; ++j) {
                auto& row = hits[static_cast<std::size_t>(k) * static_cast<std::size_t>(g.ny) + static_cast<std::size_t>(j)];
                std::sort(row.begin(), row.end());
                for (std::size_t r = 0; r + 1 < row.size(); r += 2) // inside between pairs
                    for (int i = 0; i < g.nx; ++i) {
                        const float x = g.origin.x + (static_cast<float>(i) + 0.5f) * h;
                        if (x > row[r] && x < row[r + 1]) {
                            g.solid[(static_cast<std::size_t>(k) * static_cast<std::size_t>(g.ny) + static_cast<std::size_t>(j)) * static_cast<std::size_t>(g.nx) +
                                    static_cast<std::size_t>(i)] = 1;
                            ++count;
                        }
                    }
            }
        if (count > maxCells) {
            h *= 1.15f * std::cbrt(static_cast<float>(count) / static_cast<float>(maxCells));
            continue;
        }
        if (count < 8) {
            if (cell > 0) return why = "The cubes are too big for the body: a smaller CELL", false;
            h *= 0.5f;
            continue;
        }
        break;
    }
    g.h = h;
    const long long NX = g.nx + 1, NY = g.ny + 1;
    for (int k = 0; k < g.nz; ++k)
        for (int j = 0; j < g.ny; ++j)
            for (int i = 0; i < g.nx; ++i) {
                if (!g.cellAt(i, j, k)) continue;
                std::array<int, 8> e{};
                for (int n = 0; n < 8; ++n) {
                    const int a = i + (n & 1), b = j + ((n >> 1) & 1), c = k + ((n >> 2) & 1);
                    const long long key = (static_cast<long long>(c) * NY + b) * NX + a;
                    auto [it, fresh] = g.nodeOf.emplace(key, static_cast<int>(g.nodePos.size()));
                    if (fresh) g.nodePos.push_back(g.origin + Vec3{static_cast<float>(a), static_cast<float>(b), static_cast<float>(c)} * h);
                    e[static_cast<std::size_t>(n)] = it->second;
                }
                g.elems.push_back(e);
                g.elemIdx.push_back({i, j, k});
            }
    if (g.elems.empty()) return why = "The body couldn't be filled with cubes (is it closed?)", false;
    // Surface nodes: on a face of a solid cube whose neighbour across it is empty.
    g.surface.assign(g.nodePos.size(), 0);
    for (std::size_t e = 0; e < g.elems.size(); ++e)
        for (int f = 0; f < 6; ++f)
            if (!g.cellAt(g.elemIdx[e][0] + kStep[f][0], g.elemIdx[e][1] + kStep[f][1], g.elemIdx[e][2] + kStep[f][2]))
                for (int q = 0; q < 4; ++q) g.surface[static_cast<std::size_t>(g.elems[e][static_cast<std::size_t>(kFaces[f][q])])] = 1;
    return true;
}

// Surface nodes a selection picks: FACE (the plane of the triangle nearest the point), NEAR.
std::vector<int> pickNodes(const Grid& g, const MeshData& world, const FeaSelection& s) {
    std::vector<int> nodes;
    const std::size_t N = g.nodePos.size();
    if (s.kind == FeaSelection::Kind::Near) {
        for (std::size_t n = 0; n < N; ++n)
            if (g.surface[n] && length(g.nodePos[n] - s.point) <= s.radius + 0.5f * g.h) nodes.push_back(static_cast<int>(n));
        return nodes;
    }
    float best = 1e30f;
    Vec3 q{}, nrm{0, 0, 1};
    for (std::size_t t = 0; t + 2 < world.indices.size(); t += 3) {
        const Vec3 a = world.vertices[world.indices[t]].position, b = world.vertices[world.indices[t + 1]].position,
                   c = world.vertices[world.indices[t + 2]].position;
        const Vec3 nn = cross(b - a, c - a);
        if (!(length(nn) > 0)) continue;
        const Vec3 nh = normalize(nn);
        const Vec3 onPlane = s.point - nh * dot(s.point - a, nh);
        const Vec3 c0 = cross(b - a, onPlane - a), c1 = cross(c - b, onPlane - b), c2 = cross(a - c, onPlane - c);
        const bool inside = dot(c0, nn) >= 0 && dot(c1, nn) >= 0 && dot(c2, nn) >= 0;
        const float d = inside ? std::fabs(dot(s.point - a, nh)) : std::min({length(s.point - a), length(s.point - b), length(s.point - c)});
        if (d < best) best = d, q = onPlane, nrm = nh;
    }
    for (std::size_t n = 0; n < N; ++n)
        if (g.surface[n] && std::fabs(dot(g.nodePos[n] - q, nrm)) <= 0.75f * g.h) nodes.push_back(static_cast<int>(n));
    return nodes;
}

// Preconditioned conjugate gradients for A x = b (A applied by `apply`, diagonal `diag`):
// x is the start and the answer. Iterations taken; `converged` when the residual fell
// below tol |b|.
int solvePcg(const std::function<void(const std::vector<double>&, std::vector<double>&)>& apply, const std::vector<double>& diag, const std::vector<double>& b,
             std::vector<double>& x, double tol, int maxIt, bool& converged, bool& indefinite) {
    const std::size_t n = b.size();
    std::vector<double> r(n), z(n), p(n), Ap(n);
    apply(x, Ap);
    double bn = 0;
    for (std::size_t i = 0; i < n; ++i) r[i] = b[i] - Ap[i], bn += b[i] * b[i];
    bn = std::sqrt(bn);
    converged = indefinite = false;
    if (!(bn > 0)) {
        std::fill(x.begin(), x.end(), 0.0);
        converged = true;
        return 0;
    }
    for (std::size_t i = 0; i < n; ++i) z[i] = r[i] / diag[i];
    p = z;
    double rz = 0;
    for (std::size_t i = 0; i < n; ++i) rz += r[i] * z[i];
    int it = 0;
    for (; it < maxIt; ++it) {
        apply(p, Ap);
        double pAp = 0;
        for (std::size_t i = 0; i < n; ++i) pAp += p[i] * Ap[i];
        if (!(pAp > 0)) {
            indefinite = true;
            return it;
        }
        const double alpha = rz / pAp;
        double rr = 0;
        for (std::size_t i = 0; i < n; ++i) {
            x[i] += alpha * p[i];
            r[i] -= alpha * Ap[i];
            rr += r[i] * r[i];
        }
        if (std::sqrt(rr) <= tol * bn) {
            converged = true;
            return it + 1;
        }
        double rz2 = 0;
        for (std::size_t i = 0; i < n; ++i) z[i] = r[i] / diag[i], rz2 += r[i] * z[i];
        const double beta = rz2 / rz;
        rz = rz2;
        for (std::size_t i = 0; i < n; ++i) p[i] = z[i] + beta * p[i];
    }
    return it;
}

// Eigenvalues and vectors of a small symmetric matrix (Jacobi rotations), ascending.
void symmetricEigen(std::vector<double> A, std::size_t n, std::vector<double>& values, std::vector<double>& vectors) {
    vectors.assign(n * n, 0.0);
    for (std::size_t i = 0; i < n; ++i) vectors[i * n + i] = 1;
    for (int sweep = 0; sweep < 100; ++sweep) {
        double off = 0;
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = i + 1; j < n; ++j) off += A[i * n + j] * A[i * n + j];
        if (off < 1e-30) break;
        for (std::size_t p = 0; p < n; ++p)
            for (std::size_t q = p + 1; q < n; ++q) {
                if (std::fabs(A[p * n + q]) < 1e-300) continue;
                const double theta = (A[q * n + q] - A[p * n + p]) / (2 * A[p * n + q]);
                const double t = (theta >= 0 ? 1.0 : -1.0) / (std::fabs(theta) + std::sqrt(theta * theta + 1));
                const double c = 1 / std::sqrt(t * t + 1), s = t * c;
                for (std::size_t k = 0; k < n; ++k) { // A = J^T A J
                    const double akp = A[k * n + p], akq = A[k * n + q];
                    A[k * n + p] = c * akp - s * akq, A[k * n + q] = s * akp + c * akq;
                }
                for (std::size_t k = 0; k < n; ++k) {
                    const double apk = A[p * n + k], aqk = A[q * n + k];
                    A[p * n + k] = c * apk - s * aqk, A[q * n + k] = s * apk + c * aqk;
                }
                for (std::size_t k = 0; k < n; ++k) {
                    const double vkp = vectors[k * n + p], vkq = vectors[k * n + q];
                    vectors[k * n + p] = c * vkp - s * vkq, vectors[k * n + q] = s * vkp + c * vkq;
                }
            }
    }
    std::vector<std::size_t> order(n);
    for (std::size_t i = 0; i < n; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](std::size_t x, std::size_t y) { return A[x * n + x] < A[y * n + y]; });
    std::vector<double> v2(n * n);
    values.resize(n);
    for (std::size_t c = 0; c < n; ++c) {
        values[c] = A[order[c] * n + order[c]];
        for (std::size_t r = 0; r < n; ++r) v2[r * n + c] = vectors[r * n + order[c]];
    }
    vectors = std::move(v2);
}

// Nodal values at the mesh's vertices: trilinear in the cube a vertex is in (its corners
// that are nodes), else the nearest node about.
template <typename T, typename Get>
void sampleSurface(const Grid& g, const MeshData& world, std::vector<T>& out, Get get) {
    out.assign(world.vertices.size(), T{});
    for (std::size_t v = 0; v < world.vertices.size(); ++v) {
        const Vec3 gp = (world.vertices[v].position - g.origin) / g.h;
        const long long ci = std::clamp<long long>(static_cast<long long>(std::floor(gp.x)), 0, g.nx - 1);
        const long long cj = std::clamp<long long>(static_cast<long long>(std::floor(gp.y)), 0, g.ny - 1);
        const long long ck = std::clamp<long long>(static_cast<long long>(std::floor(gp.z)), 0, g.nz - 1);
        const float fx = std::clamp(gp.x - static_cast<float>(ci), 0.0f, 1.0f), fy = std::clamp(gp.y - static_cast<float>(cj), 0.0f, 1.0f),
                    fz = std::clamp(gp.z - static_cast<float>(ck), 0.0f, 1.0f);
        double wsum = 0;
        T acc{};
        for (int n = 0; n < 8; ++n) {
            const int id = g.nodeAt(ci + (n & 1), cj + ((n >> 1) & 1), ck + ((n >> 2) & 1));
            if (id < 0) continue;
            const double w = static_cast<double>((n & 1) ? fx : 1 - fx) * ((n & 2) ? fy : 1 - fy) * ((n & 4) ? fz : 1 - fz) + 1e-9;
            wsum += w;
            acc = acc + get(static_cast<std::size_t>(id)) * static_cast<float>(w);
        }
        if (wsum <= 0) {
            float bestD = 1e30f;
            for (int dk = -1; dk <= 2; ++dk)
                for (int dj = -1; dj <= 2; ++dj)
                    for (int di = -1; di <= 2; ++di) {
                        const int id = g.nodeAt(ci + di, cj + dj, ck + dk);
                        if (id < 0) continue;
                        const float d = length(g.nodePos[static_cast<std::size_t>(id)] - world.vertices[v].position);
                        if (d < bestD) bestD = d, acc = get(static_cast<std::size_t>(id)), wsum = 1;
                    }
            if (wsum <= 0) continue;
        }
        out[v] = acc * static_cast<float>(1.0 / wsum);
    }
}

} // namespace

bool runFea(const MeshData& world, const FeaSetup& setup, FeaResult& out, std::string* why) {
    auto fail = [&](std::string w) {
        if (why) *why = std::move(w);
        return false;
    };
    out = FeaResult{};
    if (world.indices.size() < 12) return fail("The body has no faces");
    const bool heat = !setup.temperatures.empty();
    const bool loaded = !setup.loads.empty() || !setup.pressures.empty() || !setup.bearings.empty() || length(setup.gravity) > 0 ||
                        (setup.thermalStress && heat);
    const bool mechanical = loaded || setup.modes > 0;
    if (!heat && !setup.heats.empty()) return fail("Heat flows in but no face is held at a temperature (TEMP FACE x,y,z C)");
    if (mechanical && setup.fixed.empty()) return fail("Nothing holds the body: FIX a face");
    if (!mechanical && !heat) return fail("Nothing loads the body: LOAD a face with a FORCE (or PRESSURE, GRAVITY, BEARING, TEMP, MODES)");
    const FeaMaterial& mat = setup.material;
    if (!(mat.E > 0) || !(mat.nu > -0.99 && mat.nu < 0.499)) return fail("The material's E and Poisson's ratio are out of range");
    const AABB box = world.bounds();
    const float longest = std::max({box.max.x - box.min.x, box.max.y - box.min.y, box.max.z - box.min.z});
    Grid g;
    std::string err;
    if (!buildGrid(world, setup.cell, setup.modes > 0 ? std::min(setup.maxCells, 12000) : setup.maxCells, g, err)) return fail(err);
    const float h = g.h;
    const std::size_t N = g.nodePos.size(), ndof = 3 * N;
    out.cell = h;
    out.cells = static_cast<int>(g.elems.size());
    out.nodes = static_cast<int>(N);
    out.mass = mat.density * 1e-6 * static_cast<double>(h) * h * h * static_cast<double>(g.elems.size()); // g/cm3 x mm3 -> kg
    // --- heat: conduction K T = Q, temperatures held on faces ---------------------------------
    std::vector<double> T(N, setup.reference);
    if (heat) {
        const std::array<double, 64> Kc = unitConduction();
        const double cond = mat.conductivity * 1e-3 * static_cast<double>(h); // W/(mm K) x h
        std::vector<char> held(N, 0);
        std::vector<double> T0(N, 0.0);
        for (const FeaTemperature& tp : setup.temperatures) {
            const std::vector<int> nodes = pickNodes(g, world, tp.where);
            if (nodes.empty()) return fail("A TEMP picks no surface nodes there");
            for (int n : nodes) held[static_cast<std::size_t>(n)] = 1, T0[static_cast<std::size_t>(n)] = tp.celsius;
        }
        std::vector<double> Q(N, 0.0);
        for (const FeaHeat& hq : setup.heats) {
            const std::vector<int> nodes = pickNodes(g, world, hq.where);
            if (nodes.empty()) return fail("A HEAT picks no surface nodes there");
            for (int n : nodes) Q[static_cast<std::size_t>(n)] += hq.watts / static_cast<double>(nodes.size());
        }
        auto applyC = [&](const std::vector<double>& x, std::vector<double>& y) {
            std::fill(y.begin(), y.end(), 0.0);
            for (const auto& e : g.elems)
                for (int a = 0; a < 8; ++a) {
                    double v = 0;
                    for (int b = 0; b < 8; ++b) v += Kc[static_cast<std::size_t>(a * 8 + b)] * x[static_cast<std::size_t>(e[static_cast<std::size_t>(b)])];
                    y[static_cast<std::size_t>(e[static_cast<std::size_t>(a)])] += v * cond;
                }
        };
        // Held values lifted out: solve for the rest with them in place.
        std::vector<double> KT0(N);
        applyC(T0, KT0);
        std::vector<double> rhs(N), diag(N, 0.0);
        for (std::size_t n = 0; n < N; ++n) rhs[n] = held[n] ? 0.0 : Q[n] - KT0[n];
        for (const auto& e : g.elems)
            for (int a = 0; a < 8; ++a) diag[static_cast<std::size_t>(e[static_cast<std::size_t>(a)])] += Kc[static_cast<std::size_t>(a * 9)] * cond;
        for (std::size_t n = 0; n < N; ++n)
            if (held[n] || !(diag[n] > 0)) diag[n] = 1;
        auto applyFree = [&](const std::vector<double>& x, std::vector<double>& y) {
            std::vector<double> xf = x;
            for (std::size_t n = 0; n < N; ++n)
                if (held[n]) xf[n] = 0;
            applyC(xf, y);
            for (std::size_t n = 0; n < N; ++n)
                if (held[n]) y[n] = x[n];
        };
        std::vector<double> dT(N, 0.0);
        bool conv = false, indef = false;
        solvePcg(applyFree, diag, rhs, dT, 1e-9, setup.maxIterations, conv, indef);
        for (std::size_t n = 0; n < N; ++n) T[n] = held[n] ? T0[n] : dT[n];
        std::size_t lo = 0, hi = 0;
        for (std::size_t n = 0; n < N; ++n) {
            if (T[n] < T[lo]) lo = n;
            if (T[n] > T[hi]) hi = n;
        }
        out.minTemperature = T[lo], out.maxTemperature = T[hi];
        out.minTemperatureAt = g.nodePos[lo], out.maxTemperatureAt = g.nodePos[hi];
        std::vector<double> Tv;
        sampleSurface(g, world, Tv, [&](std::size_t n) { return T[n]; });
        out.surfaceTemperature.assign(Tv.begin(), Tv.end());
        if (!mechanical) {
            out.converged = conv;
            return true;
        }
    }
    // --- mechanics: fixings, stiffness ---------------------------------------------------------
    std::vector<char> fixedDof(ndof, 0);
    std::vector<int> fixedNodes;
    for (const FeaSelection& s : setup.fixed) {
        const std::vector<int> nodes = pickNodes(g, world, s);
        if (nodes.empty()) return fail("A FIX picks no surface nodes there");
        for (int n : nodes) {
            if (!fixedDof[static_cast<std::size_t>(3 * n)]) fixedNodes.push_back(n);
            for (int d = 0; d < 3; ++d) fixedDof[static_cast<std::size_t>(3 * n + d)] = 1;
        }
    }
    out.fixedNodes = static_cast<int>(fixedNodes.size());
    const std::array<double, 24 * 24> Ke = unitStiffness(mat.nu);
    const double scale = mat.E * static_cast<double>(h);
    // K x, cube by cube; large models split over the processor's threads, each adding into
    // its own copy of y.
    const unsigned threads = g.elems.size() > 4000 ? std::max(1u, std::min(8u, std::thread::hardware_concurrency())) : 1u;
    std::vector<std::vector<double>> partial(threads > 1 ? threads : 0, std::vector<double>(ndof));
    auto applyRange = [&](const std::vector<double>& x, std::vector<double>& y, std::size_t from, std::size_t to) {
        double xe[24], ye[24];
        for (std::size_t k = from; k < to; ++k) {
            const auto& e = g.elems[k];
            for (int n = 0; n < 8; ++n)
                for (int d = 0; d < 3; ++d) xe[3 * n + d] = x[static_cast<std::size_t>(3 * e[static_cast<std::size_t>(n)] + d)];
            for (int i = 0; i < 24; ++i) {
                double v = 0;
                for (int j = 0; j < 24; ++j) v += Ke[static_cast<std::size_t>(i * 24 + j)] * xe[j];
                ye[i] = v * scale;
            }
            for (int n = 0; n < 8; ++n)
                for (int d = 0; d < 3; ++d) y[static_cast<std::size_t>(3 * e[static_cast<std::size_t>(n)] + d)] += ye[3 * n + d];
        }
    };
    auto applyK = [&](const std::vector<double>& x, std::vector<double>& y) {
        std::fill(y.begin(), y.end(), 0.0);
        if (threads <= 1) applyRange(x, y, 0, g.elems.size());
        else {
            std::vector<std::thread> pool;
            const std::size_t per = (g.elems.size() + threads - 1) / threads;
            for (unsigned t = 0; t < threads; ++t) {
                pool.emplace_back([&, t] {
                    std::vector<double>& yt = partial[t];
                    std::fill(yt.begin(), yt.end(), 0.0);
                    applyRange(x, yt, std::min(g.elems.size(), t * per), std::min(g.elems.size(), (t + 1) * per));
                });
            }
            for (std::thread& th : pool) th.join();
            for (const std::vector<double>& yt : partial)
                for (std::size_t i = 0; i < ndof; ++i) y[i] += yt[i];
        }
        for (std::size_t i = 0; i < ndof; ++i)
            if (fixedDof[i]) y[i] = x[i]; // fixed: the identity
    };
    std::vector<double> diag(ndof, 0.0);
    for (const auto& e : g.elems)
        for (int n = 0; n < 8; ++n)
            for (int d = 0; d < 3; ++d) diag[static_cast<std::size_t>(3 * e[static_cast<std::size_t>(n)] + d)] += Ke[static_cast<std::size_t>((3 * n + d) * 25)] * scale;
    for (std::size_t i = 0; i < ndof; ++i)
        if (fixedDof[i] || !(diag[i] > 0)) diag[i] = 1;
    // --- natural frequencies: K x = w^2 M x (lumped mass), block inverse iteration ----------------
    if (setup.modes > 0) {
        const double rhoT = mat.density * 1e-9; // g/cm3 -> t/mm3 (mm, N, s)
        std::vector<double> M(ndof, 0.0);
        for (const auto& e : g.elems)
            for (int n = 0; n < 8; ++n)
                for (int d = 0; d < 3; ++d) M[static_cast<std::size_t>(3 * e[static_cast<std::size_t>(n)] + d)] += rhoT * static_cast<double>(h) * h * h / 8.0;
        for (std::size_t i = 0; i < ndof; ++i)
            if (fixedDof[i]) M[i] = 0;
        const std::size_t want = static_cast<std::size_t>(std::min(setup.modes, 12)), p = std::min<std::size_t>(want + 4, 2 * want + 2);
        std::vector<std::vector<double>> X(p, std::vector<double>(ndof, 0.0)), Y(p, std::vector<double>(ndof, 0.0));
        // Start: smooth, different shapes over the body.
        for (std::size_t k = 0; k < p; ++k)
            for (std::size_t n = 0; n < N; ++n) {
                const Vec3 q = (g.nodePos[n] - g.origin) / std::max(longest, 1e-6f);
                const double kk = static_cast<double>(k);
                for (int d = 0; d < 3; ++d)
                    X[k][3 * n + static_cast<std::size_t>(d)] =
                        std::sin(1.0 + kk * 1.7 + d * 2.3 + q.x * (1 + kk) + q.y * (2.0 + d) + q.z * (3.0 + static_cast<double>(k % 3)));
            }
        std::vector<double> lambda(p, 0.0), prev(p, 0.0);
        int sweeps = 0;
        for (; sweeps < 60; ++sweeps) {
            for (std::size_t k = 0; k < p; ++k) { // Y = K^-1 M X
                std::vector<double> b(ndof);
                for (std::size_t i = 0; i < ndof; ++i) b[i] = M[i] * X[k][i];
                bool conv = false, indef = false;
                solvePcg(applyK, diag, b, Y[k], 1e-6, setup.maxIterations, conv, indef);
                if (indef) return fail("The body isn't held firmly enough to vibrate on its own: FIX more, or a whole face");
            }
            // Rayleigh-Ritz on Y: Kr = Y^T K Y, Mr = Y^T M Y.
            std::vector<std::vector<double>> KY(p, std::vector<double>(ndof));
            for (std::size_t k = 0; k < p; ++k) applyK(Y[k], KY[k]);
            std::vector<double> Kr(p * p), Mr(p * p);
            for (std::size_t a = 0; a < p; ++a)
                for (std::size_t b = a; b < p; ++b) {
                    double kv = 0, mv = 0;
                    for (std::size_t i = 0; i < ndof; ++i) {
                        if (fixedDof[i]) continue;
                        kv += Y[a][i] * KY[b][i], mv += Y[a][i] * M[i] * Y[b][i];
                    }
                    Kr[a * p + b] = Kr[b * p + a] = kv, Mr[a * p + b] = Mr[b * p + a] = mv;
                }
            // Mr = L L^T; solve L^-1 Kr L^-T, then back.
            std::vector<double> L(p * p, 0.0);
            for (std::size_t i = 0; i < p; ++i)
                for (std::size_t j = 0; j <= i; ++j) {
                    double sum = Mr[i * p + j];
                    for (std::size_t k = 0; k < j; ++k) sum -= L[i * p + k] * L[j * p + k];
                    if (i == j) {
                        if (!(sum > 0)) return fail("The vibration modes could not be separated (a finer CELL may help)");
                        L[i * p + i] = std::sqrt(sum);
                    } else L[i * p + j] = sum / L[j * p + j];
                }
            auto lsolve = [&](std::vector<double> v) { // L^-1 v
                for (std::size_t i = 0; i < p; ++i) {
                    for (std::size_t k = 0; k < i; ++k) v[i] -= L[i * p + k] * v[k];
                    v[i] /= L[i * p + i];
                }
                return v;
            };
            std::vector<double> A(p * p);
            for (std::size_t c = 0; c < p; ++c) { // A = L^-1 Kr L^-T, column by column
                std::vector<double> col(p);
                for (std::size_t r = 0; r < p; ++r) col[r] = Kr[r * p + c];
                const std::vector<double> t1 = lsolve(col);
                for (std::size_t r = 0; r < p; ++r) A[r * p + c] = t1[r];
            }
            for (std::size_t r = 0; r < p; ++r) { // then rows: A <- A L^-T
                std::vector<double> row(p);
                for (std::size_t c = 0; c < p; ++c) row[c] = A[r * p + c];
                const std::vector<double> t2 = lsolve(row);
                for (std::size_t c = 0; c < p; ++c) A[r * p + c] = t2[c];
            }
            std::vector<double> vals, vecs;
            symmetricEigen(A, p, vals, vecs);
            // Q = L^-T vecs: back substitution with L^T.
            std::vector<double> Qm(p * p);
            for (std::size_t c = 0; c < p; ++c) {
                std::vector<double> v(p);
                for (std::size_t r = 0; r < p; ++r) v[r] = vecs[r * p + c];
                for (std::size_t i = p; i-- > 0;) {
                    for (std::size_t k = i + 1; k < p; ++k) v[i] -= L[k * p + i] * v[k];
                    v[i] /= L[i * p + i];
                }
                for (std::size_t r = 0; r < p; ++r) Qm[r * p + c] = v[r];
            }
            for (std::size_t c = 0; c < p; ++c) {
                std::fill(X[c].begin(), X[c].end(), 0.0);
                for (std::size_t r = 0; r < p; ++r)
                    for (std::size_t i = 0; i < ndof; ++i) X[c][i] += Y[r][i] * Qm[r * p + c];
            }
            prev = lambda;
            lambda = vals;
            bool settled = sweeps > 0;
            for (std::size_t k = 0; k < want && settled; ++k) settled = std::fabs(lambda[k] - prev[k]) <= 1e-5 * std::fabs(lambda[k]);
            for (std::size_t k = 0; k < p; ++k) Y[k] = X[k]; // the next solves start from here
            if (settled) break;
        }
        out.iterations = sweeps + 1;
        out.converged = sweeps < 60;
        for (std::size_t k = 0; k < want; ++k) {
            out.frequencies.push_back(std::sqrt(std::max(0.0, lambda[k])) / (2 * 3.14159265358979323846));
            double big = 0;
            for (std::size_t n = 0; n < N; ++n)
                big = std::max(big, std::sqrt(X[k][3 * n] * X[k][3 * n] + X[k][3 * n + 1] * X[k][3 * n + 1] + X[k][3 * n + 2] * X[k][3 * n + 2]));
            std::vector<Vec3> shape;
            sampleSurface(g, world, shape, [&](std::size_t n) {
                return Vec3{static_cast<float>(X[k][3 * n] / big), static_cast<float>(X[k][3 * n + 1] / big), static_cast<float>(X[k][3 * n + 2] / big)};
            });
            out.modeShapes.push_back(std::move(shape));
        }
        if (!loaded) return true;
    }
    // --- static loads ---------------------------------------------------------------------------
    std::vector<double> f(ndof, 0.0);
    int loadedNodes = 0;
    for (const FeaLoad& l : setup.loads) {
        const std::vector<int> nodes = pickNodes(g, world, l.where);
        if (nodes.empty()) return fail("A LOAD picks no surface nodes there");
        for (int n : nodes)
            for (int d = 0; d < 3; ++d) f[static_cast<std::size_t>(3 * n + d)] += static_cast<double>((&l.force.x)[d]) / static_cast<double>(nodes.size());
        loadedNodes += static_cast<int>(nodes.size());
    }
    for (const FeaPressure& pr : setup.pressures) { // on each open cube face all of whose corners are picked
        const std::vector<int> nodes = pickNodes(g, world, pr.where);
        std::vector<char> in(N, 0);
        for (int n : nodes) in[static_cast<std::size_t>(n)] = 1;
        int faces = 0;
        for (std::size_t e = 0; e < g.elems.size(); ++e)
            for (int fc = 0; fc < 6; ++fc) {
                if (g.cellAt(g.elemIdx[e][0] + kStep[fc][0], g.elemIdx[e][1] + kStep[fc][1], g.elemIdx[e][2] + kStep[fc][2])) continue;
                bool all = true;
                for (int q = 0; q < 4 && all; ++q) all = in[static_cast<std::size_t>(g.elems[e][static_cast<std::size_t>(kFaces[fc][q])])] != 0;
                if (!all) continue;
                ++faces;
                const double per = pr.mpa * static_cast<double>(h) * h / 4.0;
                for (int q = 0; q < 4; ++q)
                    for (int d = 0; d < 3; ++d)
                        f[static_cast<std::size_t>(3 * g.elems[e][static_cast<std::size_t>(kFaces[fc][q])] + d)] -= per * kStep[fc][d]; // pushing in
            }
        if (!faces) return fail("A PRESSURE picks no face there");
        loadedNodes += static_cast<int>(nodes.size());
    }
    for (const FeaBearing& br : setup.bearings) { // cosine-spread over the side the force pushes on
        const Vec3 dir = normalize(br.force);
        std::vector<std::pair<int, double>> w;
        double total = 0;
        for (std::size_t n = 0; n < N; ++n) {
            if (!g.surface[n] || length(g.nodePos[n] - br.center) > br.radius + 0.5f * h) continue;
            Vec3 rad = g.nodePos[n] - br.center;
            rad = rad - dir * 0.0f;
            const double c = dot(normalize(rad), dir);
            if (c > 0) w.push_back({static_cast<int>(n), c}), total += c;
        }
        if (!(total > 0)) return fail("A BEARING finds no hole surface round its centre");
        for (const auto& [n, c] : w)
            for (int d = 0; d < 3; ++d) f[static_cast<std::size_t>(3 * n + d)] += static_cast<double>((&br.force.x)[d]) * c / total;
        loadedNodes += static_cast<int>(w.size());
    }
    if (length(setup.gravity) > 0) { // its own weight: each cube's, an eighth to each corner
        const double perNode = mat.density * 1e-9 * static_cast<double>(h) * h * h * 9810.0 / 8.0; // t x mm/s2 = N
        for (const auto& e : g.elems)
            for (int n = 0; n < 8; ++n)
                for (int d = 0; d < 3; ++d) f[static_cast<std::size_t>(3 * e[static_cast<std::size_t>(n)] + d)] += perNode * static_cast<double>((&setup.gravity.x)[d]);
    }
    // Thermal expansion: each cube's free strain alpha (T - Tref) held back by its
    // neighbours: nodal forces sigma0 h^2 / 4 outward along each axis (exact for a cube).
    const double lam = mat.E * mat.nu / ((1 + mat.nu) * (1 - 2 * mat.nu)), mu = mat.E / (2 * (1 + mat.nu));
    const double alpha = mat.expansion * 1e-6;
    std::vector<double> cubeDT(g.elems.size(), 0.0);
    if (setup.thermalStress && heat) {
        for (std::size_t e = 0; e < g.elems.size(); ++e) {
            double t = 0;
            for (int n = 0; n < 8; ++n) t += T[static_cast<std::size_t>(g.elems[e][static_cast<std::size_t>(n)])];
            cubeDT[e] = t / 8 - setup.reference;
            const double s0 = (3 * lam + 2 * mu) * alpha * cubeDT[e] * static_cast<double>(h) * h / 4.0;
            for (int n = 0; n < 8; ++n)
                for (int d = 0; d < 3; ++d)
                    f[static_cast<std::size_t>(3 * g.elems[e][static_cast<std::size_t>(n)] + d)] += s0 * ((n >> d) & 1 ? 1.0 : -1.0);
        }
    }
    out.loadedNodes = loadedNodes;
    for (std::size_t i = 0; i < ndof; ++i)
        if (fixedDof[i]) f[i] = 0;
    double fn = 0;
    for (double v : f) fn += v * v;
    if (!(fn > 0)) return fail("The loads add up to nothing");
    std::vector<double> u(ndof, 0.0);
    bool conv = false, indef = false;
    const int it = solvePcg(applyK, diag, f, u, setup.tolerance, setup.maxIterations, conv, indef);
    if (indef) return fail("The body isn't held firmly enough: it could move or turn freely (FIX more, or a whole face)");
    double umax = 0;
    std::size_t umaxAt = 0;
    for (std::size_t n = 0; n < N; ++n) {
        const double m = std::sqrt(u[3 * n] * u[3 * n] + u[3 * n + 1] * u[3 * n + 1] + u[3 * n + 2] * u[3 * n + 2]);
        if (m > umax) umax = m, umaxAt = n;
    }
    if (!std::isfinite(umax) || umax > 1e3 * static_cast<double>(longest)) return fail("The body isn't held firmly enough: it could move or turn freely");
    // Stress at each cube's corners (D (B u - thermal strain) there, then von Mises), averaged
    // over the cubes meeting at a node: corners lie on the surface, where bending stress peaks.
    std::vector<double> nodeStress(N, 0.0);
    std::vector<int> nodeCount(N, 0);
    for (std::size_t e = 0; e < g.elems.size(); ++e)
        for (int c = 0; c < 8; ++c) {
            const double xi[3] = {static_cast<double>(c & 1), static_cast<double>((c >> 1) & 1), static_cast<double>((c >> 2) & 1)};
            double eps[6] = {};
            for (int n = 0; n < 8; ++n) {
                const double sgn[3] = {static_cast<double>(n & 1), static_cast<double>((n >> 1) & 1), static_cast<double>((n >> 2) & 1)};
                double fc[3], dN[3];
                for (int k = 0; k < 3; ++k) fc[k] = sgn[k] > 0.5 ? xi[k] : 1 - xi[k];
                for (int k = 0; k < 3; ++k) {
                    double d = (sgn[k] > 0.5 ? 1.0 : -1.0) / static_cast<double>(h);
                    for (int m = 0; m < 3; ++m)
                        if (m != k) d *= fc[m];
                    dN[k] = d;
                }
                const std::size_t id = static_cast<std::size_t>(g.elems[e][static_cast<std::size_t>(n)]);
                const double ux = u[3 * id], uy = u[3 * id + 1], uz = u[3 * id + 2];
                eps[0] += dN[0] * ux, eps[1] += dN[1] * uy, eps[2] += dN[2] * uz;
                eps[3] += dN[1] * ux + dN[0] * uy, eps[4] += dN[2] * uy + dN[1] * uz, eps[5] += dN[2] * ux + dN[0] * uz;
            }
            const double th = alpha * cubeDT[e];
            for (int k = 0; k < 3; ++k) eps[k] -= th;
            const double tr = eps[0] + eps[1] + eps[2];
            const double sx = lam * tr + 2 * mu * eps[0], sy = lam * tr + 2 * mu * eps[1], sz = lam * tr + 2 * mu * eps[2];
            const double txy = mu * eps[3], tyz = mu * eps[4], txz = mu * eps[5];
            const double vm = std::sqrt(0.5 * ((sx - sy) * (sx - sy) + (sy - sz) * (sy - sz) + (sz - sx) * (sz - sx)) + 3 * (txy * txy + tyz * tyz + txz * txz));
            const std::size_t node = static_cast<std::size_t>(g.elems[e][static_cast<std::size_t>(c)]);
            nodeStress[node] += vm, ++nodeCount[node];
        }
    for (std::size_t n = 0; n < N; ++n)
        if (nodeCount[n]) nodeStress[n] /= nodeCount[n];
    // The peak away from the held nodes, where holding everything still overstates it.
    std::size_t smaxAt = 0;
    double smax = -1;
    for (std::size_t n = 0; n < N; ++n)
        if (!fixedDof[3 * n] && nodeStress[n] > smax) smax = nodeStress[n], smaxAt = n;
    if (smax < 0) smax = 0;
    out.iterations = it;
    out.converged = conv;
    out.maxStress = smax;
    out.maxStressAt = g.nodePos[smaxAt];
    out.maxDisplacement = umax;
    out.maxDisplacementAt = g.nodePos[umaxAt];
    out.safetyFactor = out.maxStress > 0 ? mat.yield / out.maxStress : 0;
    std::vector<double> sv;
    sampleSurface(g, world, sv, [&](std::size_t n) { return nodeStress[n]; });
    out.surfaceStress.assign(sv.begin(), sv.end());
    sampleSurface(g, world, out.surfaceDisplacement, [&](std::size_t n) {
        return Vec3{static_cast<float>(u[3 * n]), static_cast<float>(u[3 * n + 1]), static_cast<float>(u[3 * n + 2])};
    });
    return true;
}

bool runTopology(const MeshData& world, const TopologySetup& setup, TopologyResult& out, std::string* why) {
    auto fail = [&](std::string w) {
        if (why) *why = std::move(w);
        return false;
    };
    out = {};
    if (!(setup.volume > 0.02) || setup.volume > 0.95) return fail("VOLUME is the share of material kept: 0.02 to 0.95");
    if (setup.fea.fixed.empty()) return fail("FIX it somewhere (FIX FACE x,y,z or FIX NEAR x,y,z r)");
    if (setup.fea.loads.empty() && setup.fea.bearings.empty() && !(length(setup.fea.gravity) > 0)) return fail("LOAD it (LOAD ... FORCE fx,fy,fz, BEARING or GRAVITY)");
    Grid g;
    std::string w;
    if (!buildGrid(world, setup.fea.cell, setup.maxCells, g, w)) return fail(w);
    const std::size_t E = g.elems.size(), N = g.nodePos.size(), ndof = 3 * N;
    const FeaMaterial& mat = setup.fea.material;
    const double h = g.h, scale = mat.E * h;
    const std::array<double, 24 * 24> Ke = unitStiffness(mat.nu);
    // Fixings and loads, as the analysis takes them.
    std::vector<char> fixedDof(ndof, 0), touched(N, 0);
    for (const FeaSelection& sel : setup.fea.fixed) {
        const std::vector<int> nodes = pickNodes(g, world, sel);
        if (nodes.empty()) return fail("A FIX picks no surface nodes there");
        for (int n : nodes) {
            touched[static_cast<std::size_t>(n)] = 1;
            for (int d = 0; d < 3; ++d) fixedDof[static_cast<std::size_t>(3 * n + d)] = 1;
        }
    }
    std::vector<double> f(ndof, 0.0);
    for (const FeaLoad& l : setup.fea.loads) {
        const std::vector<int> nodes = pickNodes(g, world, l.where);
        if (nodes.empty()) return fail("A LOAD picks no surface nodes there");
        for (int n : nodes) {
            touched[static_cast<std::size_t>(n)] = 1;
            for (int d = 0; d < 3; ++d) f[static_cast<std::size_t>(3 * n + d)] += static_cast<double>((&l.force.x)[d]) / static_cast<double>(nodes.size());
        }
    }
    for (const FeaBearing& br : setup.fea.bearings) {
        const Vec3 dir = normalize(br.force);
        std::vector<std::pair<int, double>> wts;
        double total = 0;
        for (std::size_t n = 0; n < N; ++n) {
            if (!g.surface[n] || length(g.nodePos[n] - br.center) > br.radius + 0.5f * g.h) continue;
            const double c = dot(normalize(g.nodePos[n] - br.center), dir);
            if (c > 0) wts.push_back({static_cast<int>(n), c}), total += c;
        }
        if (!(total > 0)) return fail("A BEARING finds no hole surface round its centre");
        for (const auto& [n, c] : wts) {
            touched[static_cast<std::size_t>(n)] = 1;
            for (int d = 0; d < 3; ++d) f[static_cast<std::size_t>(3 * n + d)] += static_cast<double>((&br.force.x)[d]) * c / total;
        }
    }
    const bool gravity = length(setup.fea.gravity) > 0;
    // Cubes at the fixings and loads stay solid.
    std::vector<char> passive(E, 0);
    for (std::size_t e = 0; e < E; ++e)
        for (int n = 0; n < 8 && !passive[e]; ++n) passive[e] = touched[static_cast<std::size_t>(g.elems[e][static_cast<std::size_t>(n)])];
    std::size_t active = 0;
    for (char c : passive) active += !c;
    if (active == 0) return fail("Nothing is left to take away: the fixings and loads cover the body");
    // Neighbours for the sensitivity filter, by the cubes' grid places.
    std::unordered_map<long long, int> elemAt;
    auto key = [&](int i, int j, int k) { return (static_cast<long long>(k) * g.ny + j) * g.nx + i; };
    for (std::size_t e = 0; e < E; ++e) elemAt[key(g.elemIdx[e][0], g.elemIdx[e][1], g.elemIdx[e][2])] = static_cast<int>(e);
    const int reach = static_cast<int>(std::ceil(setup.filter)) - 1;
    std::vector<std::vector<std::pair<int, double>>> nbr(E);
    for (std::size_t e = 0; e < E; ++e) {
        const auto [i, j, k] = g.elemIdx[e];
        for (int a = -reach; a <= reach; ++a)
            for (int b = -reach; b <= reach; ++b)
                for (int c = -reach; c <= reach; ++c) {
                    const double wgt = setup.filter - std::sqrt(static_cast<double>(a * a + b * b + c * c));
                    if (wgt <= 0) continue;
                    const auto it = elemAt.find(key(i + a, j + b, k + c));
                    if (it != elemAt.end()) nbr[e].push_back({it->second, wgt});
                }
    }
    // The densities: the passive ones solid, the rest evenly to the volume.
    const double target = setup.volume * static_cast<double>(E);
    std::vector<double> x(E, 1.0);
    {
        const double passiveCount = static_cast<double>(E - active);
        const double start = std::clamp((target - passiveCount) / static_cast<double>(active), 0.001, 1.0);
        for (std::size_t e = 0; e < E; ++e)
            if (!passive[e]) x[e] = start;
    }
    constexpr double kEmin = 1e-3; // empty cubes keep a little stiffness: the iterative solve stays well conditioned
    auto stiff = [&](double d) { return kEmin + std::pow(d, setup.penalty) * (1 - kEmin); };
    std::vector<double> u(ndof, 0.0), ue(24), diag(ndof);
    std::vector<double> escale(E); // each cube's stiffness this step
    const unsigned threads = E > 1500 ? std::max(1u, std::min(8u, std::thread::hardware_concurrency())) : 1u;
    std::vector<std::vector<double>> partial(threads > 1 ? threads : 0, std::vector<double>(ndof));
    auto applyRange = [&](const std::vector<double>& xin, std::vector<double>& y, std::size_t from, std::size_t to) {
        double xe[24];
        for (std::size_t e = from; e < to; ++e) {
            const auto& el = g.elems[e];
            const double s = escale[e];
            for (int n = 0; n < 8; ++n)
                for (int d = 0; d < 3; ++d) xe[3 * n + d] = xin[static_cast<std::size_t>(3 * el[static_cast<std::size_t>(n)] + d)];
            for (int a = 0; a < 24; ++a) {
                double v = 0;
                for (int b = 0; b < 24; ++b) v += Ke[static_cast<std::size_t>(a * 24 + b)] * xe[b];
                y[static_cast<std::size_t>(3 * el[static_cast<std::size_t>(a / 3)] + a % 3)] += v * s;
            }
        }
    };
    auto applyK = [&](const std::vector<double>& xin, std::vector<double>& y) {
        std::fill(y.begin(), y.end(), 0.0);
        if (threads <= 1) applyRange(xin, y, 0, E);
        else {
            std::vector<std::thread> pool;
            const std::size_t per = (E + threads - 1) / threads;
            for (unsigned t = 0; t < threads; ++t)
                pool.emplace_back([&, t] {
                    std::vector<double>& yt = partial[t];
                    std::fill(yt.begin(), yt.end(), 0.0);
                    applyRange(xin, yt, std::min<std::size_t>(E, t * per), std::min<std::size_t>(E, (t + 1) * per));
                });
            for (std::thread& th : pool) th.join();
            for (const std::vector<double>& yt : partial)
                for (std::size_t i = 0; i < ndof; ++i) y[i] += yt[i];
        }
        for (std::size_t i = 0; i < ndof; ++i)
            if (fixedDof[i]) y[i] = xin[i];
    };
    std::vector<double> dc(E), dcf(E), xnew(E);
    for (int it = 0; it < setup.iterations; ++it) {
        // Loads (its own weight follows the material left).
        std::vector<double> load = f;
        if (gravity) {
            const double perNode = mat.density * 1e-9 * h * h * h * 9810.0 / 8.0;
            for (std::size_t e = 0; e < E; ++e)
                for (int n = 0; n < 8; ++n)
                    for (int d = 0; d < 3; ++d)
                        load[static_cast<std::size_t>(3 * g.elems[e][static_cast<std::size_t>(n)] + d)] += perNode * x[e] * static_cast<double>((&setup.fea.gravity.x)[d]);
        }
        for (std::size_t i = 0; i < ndof; ++i)
            if (fixedDof[i]) load[i] = 0;
        for (std::size_t e = 0; e < E; ++e) escale[e] = stiff(x[e]) * scale;
        std::fill(diag.begin(), diag.end(), 0.0);
        for (std::size_t e = 0; e < E; ++e)
            for (int n = 0; n < 8; ++n)
                for (int d = 0; d < 3; ++d)
                    diag[static_cast<std::size_t>(3 * g.elems[e][static_cast<std::size_t>(n)] + d)] += Ke[static_cast<std::size_t>((3 * n + d) * 25)] * scale * stiff(x[e]);
        for (std::size_t i = 0; i < ndof; ++i)
            if (fixedDof[i] || !(diag[i] > 0)) diag[i] = 1;
        bool conv = false, indef = false;
        const int cgIts = solvePcg(applyK, diag, load, u, 1e-5, 4000, conv, indef); // from the last step's answer
        out.solverIterations += cgIts;
        if (indef) return fail("The fixings leave it free to move");
        // Compliance and its sensitivities.
        double c = 0;
        for (std::size_t i = 0; i < ndof; ++i) c += load[i] * u[i];
        for (std::size_t e = 0; e < E; ++e) {
            const auto& el = g.elems[e];
            for (int n = 0; n < 8; ++n)
                for (int d = 0; d < 3; ++d) ue[static_cast<std::size_t>(3 * n + d)] = u[static_cast<std::size_t>(3 * el[static_cast<std::size_t>(n)] + d)];
            double ek = 0;
            for (int a = 0; a < 24; ++a) {
                double v = 0;
                for (int b = 0; b < 24; ++b) v += Ke[static_cast<std::size_t>(a * 24 + b)] * ue[static_cast<std::size_t>(b)];
                ek += ue[static_cast<std::size_t>(a)] * v;
            }
            dc[e] = -setup.penalty * std::pow(x[e], setup.penalty - 1) * (1 - kEmin) * ek * scale;
        }
        if (it == 0) out.startCompliance = c;
        out.history.push_back(c);
        out.compliance = c;
        // Filtered (Sigmund's sensitivity filter).
        for (std::size_t e = 0; e < E; ++e) {
            double sum = 0, wsum = 0;
            for (const auto& [o, wgt] : nbr[e]) sum += wgt * x[static_cast<std::size_t>(o)] * dc[static_cast<std::size_t>(o)], wsum += wgt;
            dcf[e] = sum / (std::max(x[e], 1e-3) * wsum);
        }
        // Optimality criteria: the multiplier that keeps the volume, by halving.
        double l1 = 0, l2 = 1e12, change = 0;
        const double move = 0.2;
        while ((l2 - l1) / (l1 + l2 + 1e-30) > 1e-6) {
            const double lm = 0.5 * (l1 + l2);
            double vol = 0;
            for (std::size_t e = 0; e < E; ++e) {
                if (passive[e]) {
                    xnew[e] = 1;
                } else {
                    const double be = std::sqrt(std::max(0.0, -dcf[e] / lm));
                    xnew[e] = std::clamp(x[e] * be, std::max(0.001, x[e] - move), std::min(1.0, x[e] + move));
                }
                vol += xnew[e];
            }
            if (vol > target) l1 = lm;
            else l2 = lm;
        }
        for (std::size_t e = 0; e < E; ++e) change = std::max(change, std::fabs(xnew[e] - x[e]));
        x = xnew;
        out.iterations = it + 1;
        if (change < 0.01 && it > 4) {
            out.converged = true;
            break;
        }
    }
    double vol = 0;
    for (double d : x) vol += d;
    out.volume = vol / static_cast<double>(E);
    out.cell = g.h;
    out.cells = static_cast<int>(E);
    // Only the material joined to the fixings and loads (through cubes over one half) stays:
    // floating islands carry nothing.
    {
        std::vector<char> reached(E, 0);
        std::vector<int> stack;
        for (std::size_t e = 0; e < E; ++e)
            if (passive[e] && x[e] > 0.5) reached[e] = 1, stack.push_back(static_cast<int>(e));
        while (!stack.empty()) {
            const int e = stack.back();
            stack.pop_back();
            const auto [i, j, k] = g.elemIdx[static_cast<std::size_t>(e)];
            const int d[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
            for (const auto& s3 : d) {
                const auto it = elemAt.find(key(i + s3[0], j + s3[1], k + s3[2]));
                if (it == elemAt.end()) continue;
                const auto o = static_cast<std::size_t>(it->second);
                if (!reached[o] && x[o] > 0.5) reached[o] = 1, stack.push_back(it->second);
            }
        }
        for (std::size_t e = 0; e < E; ++e)
            if (!reached[e]) x[e] = std::min(x[e], 0.49);
    }
    // The shape: where the density passes one half (outside the body: none).
    const int px = g.nx + 2, py = g.ny + 2, pz = g.nz + 2;
    std::vector<float> field(static_cast<std::size_t>(px) * py * pz, 0.5f);
    for (std::size_t e = 0; e < E; ++e) {
        const auto [i, j, k] = g.elemIdx[e];
        field[(static_cast<std::size_t>(i + 1) * static_cast<std::size_t>(py) + static_cast<std::size_t>(j + 1)) * static_cast<std::size_t>(pz) + static_cast<std::size_t>(k + 1)] =
            static_cast<float>(0.5 - x[e]);
    }
    out.mesh = marchTets(field, px, py, pz, g.origin - Vec3{g.h, g.h, g.h}, g.h, true);
    if (out.mesh.indices.empty()) return fail("No shape came out (try a larger VOLUME)");
    {
        // Pieces apart from the rest (a strut a cube thin meshes in bits): only those with at least
        // a twentieth of the largest's volume stay.
        MeshData& m = out.mesh;
        std::vector<int> parent(m.vertices.size());
        for (std::size_t v = 0; v < parent.size(); ++v) parent[v] = static_cast<int>(v);
        std::function<int(int)> root = [&](int v) { return parent[static_cast<std::size_t>(v)] == v ? v : parent[static_cast<std::size_t>(v)] = root(parent[static_cast<std::size_t>(v)]); };
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
            for (int q = 1; q < 3; ++q) {
                const int a = root(static_cast<int>(m.indices[t])), b = root(static_cast<int>(m.indices[t + static_cast<std::size_t>(q)]));
                if (a != b) parent[static_cast<std::size_t>(a)] = b;
            }
        std::unordered_map<int, double> pieceVol;
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
            const Vec3 a = m.vertices[m.indices[t]].position, b = m.vertices[m.indices[t + 1]].position, c = m.vertices[m.indices[t + 2]].position;
            pieceVol[root(static_cast<int>(m.indices[t]))] += static_cast<double>(dot(a, cross(b, c))) / 6.0;
        }
        double largest = 0;
        for (const auto& [r, v] : pieceVol) largest = std::max(largest, v);
        std::vector<std::uint32_t> keep;
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
            if (pieceVol[root(static_cast<int>(m.indices[t]))] >= 0.05 * largest) keep.insert(keep.end(), {m.indices[t], m.indices[t + 1], m.indices[t + 2]});
        m.indices = std::move(keep);
    }
    smoothVertexNormals(out.mesh);
    out.mesh.computeFeatureEdges(40.0f);
    return true;
}

} // namespace sicdm


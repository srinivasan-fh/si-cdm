#include "core/Fea.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
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
            for (int k = 0; k < 3; ++k) f[k] = s[k] ? xi[k] : 1 - xi[k];
            for (int k = 0; k < 3; ++k) {
                double d = s[k] ? 1 : -1;
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
        {"steel", 200000, 0.30, 250, 7.85},     {"stainless", 193000, 0.29, 215, 8.0}, {"aluminium", 69000, 0.33, 275, 2.70},
        {"titanium", 114000, 0.34, 880, 4.43},  {"brass", 100000, 0.34, 200, 8.5},     {"copper", 117000, 0.34, 70, 8.96},
        {"abs", 2100, 0.35, 40, 1.04},          {"pla", 3500, 0.36, 50, 1.24},         {"petg", 2100, 0.38, 50, 1.27},
        {"nylon", 2700, 0.39, 50, 1.14},
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

bool runFea(const MeshData& world, const FeaSetup& setup, FeaResult& out, std::string* why) {
    auto fail = [&](std::string w) {
        if (why) *why = std::move(w);
        return false;
    };
    out = FeaResult{};
    if (world.indices.size() < 12) return fail("The body has no faces");
    if (setup.fixed.empty()) return fail("Nothing holds the body: FIX a face");
    if (setup.loads.empty()) return fail("Nothing loads the body: LOAD a face with a FORCE");
    const FeaMaterial& mat = setup.material;
    if (!(mat.E > 0) || !(mat.nu > -0.99 && mat.nu < 0.499)) return fail("The material's E and Poisson's ratio are out of range");
    const AABB box = world.bounds();
    const Vec3 size = box.max - box.min;
    const float longest = std::max({size.x, size.y, size.z});
    if (!(longest > 0)) return fail("The body has no size");
    float h = setup.cell > 0 ? setup.cell : longest / 40.0f;
    // Fill the body: along each row of cube centres (in x), the stretches inside the surface.
    std::vector<char> solid;
    int nx = 0, ny = 0, nz = 0;
    Vec3 origin;
    for (int attempt = 0; attempt < 12; ++attempt) {
        // Cube faces on the bounding box's: a block's faces fall on cube faces.
        origin = box.min;
        nx = std::max(1, static_cast<int>(std::ceil(size.x / h - 1e-4f))), ny = std::max(1, static_cast<int>(std::ceil(size.y / h - 1e-4f))),
        nz = std::max(1, static_cast<int>(std::ceil(size.z / h - 1e-4f)));
        const long long total = static_cast<long long>(nx) * ny * nz;
        if (total > 40LL * setup.maxCells) { // too fine to even try
            h *= 1.25f;
            continue;
        }
        solid.assign(static_cast<std::size_t>(total), 0);
        std::vector<std::vector<float>> hits(static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz));
        for (std::size_t t = 0; t + 2 < world.indices.size(); t += 3) {
            const Vec3 a = world.vertices[world.indices[t]].position, b = world.vertices[world.indices[t + 1]].position,
                       c = world.vertices[world.indices[t + 2]].position;
            const float y0 = std::min({a.y, b.y, c.y}), y1 = std::max({a.y, b.y, c.y}), z0 = std::min({a.z, b.z, c.z}), z1 = std::max({a.z, b.z, c.z});
            const int j0 = std::max(0, static_cast<int>(std::ceil((y0 - origin.y) / h - 0.5f))), j1 = std::min(ny - 1, static_cast<int>(std::floor((y1 - origin.y) / h - 0.5f)));
            const int k0 = std::max(0, static_cast<int>(std::ceil((z0 - origin.z) / h - 0.5f))), k1 = std::min(nz - 1, static_cast<int>(std::floor((z1 - origin.z) / h - 0.5f)));
            for (int k = k0; k <= k1; ++k)
                for (int j = j0; j <= j1; ++j) {
                    // A tiny shift keeps rays off edges and vertices.
                    const double py = origin.y + (j + 0.5) * h + 1.3e-5 * h, pz = origin.z + (k + 0.5) * h + 0.7e-5 * h;
                    const double d1 = (b.y - a.y) * (pz - a.z) - (b.z - a.z) * (py - a.y);
                    const double d2 = (c.y - b.y) * (pz - b.z) - (c.z - b.z) * (py - b.y);
                    const double d3 = (a.y - c.y) * (pz - c.z) - (a.z - c.z) * (py - c.y);
                    if (!((d1 >= 0 && d2 >= 0 && d3 >= 0) || (d1 <= 0 && d2 <= 0 && d3 <= 0))) continue;
                    // Where the row meets the triangle's plane.
                    const Vec3 n = cross(b - a, c - a);
                    if (std::fabs(n.x) < 1e-20f) continue;
                    const double x = a.x - (n.y * (py - a.y) + n.z * (pz - a.z)) / n.x;
                    hits[static_cast<std::size_t>(k) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(j)].push_back(static_cast<float>(x));
                }
        }
        int count = 0;
        for (int k = 0; k < nz; ++k)
            for (int j = 0; j < ny; ++j) {
                auto& row = hits[static_cast<std::size_t>(k) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(j)];
                std::sort(row.begin(), row.end());
                for (std::size_t r = 0; r + 1 < row.size(); r += 2) // inside between pairs
                    for (int i = 0; i < nx; ++i) {
                        const float x = origin.x + (static_cast<float>(i) + 0.5f) * h;
                        if (x > row[r] && x < row[r + 1]) {
                            solid[(static_cast<std::size_t>(k) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(j)) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)] = 1;
                            ++count;
                        }
                    }
            }
        if (count > setup.maxCells) {
            h *= 1.15f * std::cbrt(static_cast<float>(count) / static_cast<float>(setup.maxCells));
            continue;
        }
        if (count < 8) {
            if (setup.cell > 0) return fail("The cubes are too big for the body: a smaller CELL");
            h *= 0.5f;
            continue;
        }
        break;
    }
    auto cellAt = [&](int i, int j, int k) -> bool {
        if (i < 0 || j < 0 || k < 0 || i >= nx || j >= ny || k >= nz) return false;
        return solid[(static_cast<std::size_t>(k) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(j)) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)] != 0;
    };
    // Nodes: the corners of solid cubes, numbered as met.
    const long long NX = nx + 1, NY = ny + 1;
    std::unordered_map<long long, int> nodeOf;
    std::vector<std::array<int, 8>> elems;
    std::vector<Vec3> nodePos;
    std::vector<std::array<int, 3>> elemIdx;
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                if (!cellAt(i, j, k)) continue;
                std::array<int, 8> e{};
                for (int n = 0; n < 8; ++n) {
                    const int a = i + (n & 1), b = j + ((n >> 1) & 1), c = k + ((n >> 2) & 1);
                    const long long key = (static_cast<long long>(c) * NY + b) * NX + a;
                    auto [it, fresh] = nodeOf.emplace(key, static_cast<int>(nodePos.size()));
                    if (fresh) nodePos.push_back(origin + Vec3{static_cast<float>(a), static_cast<float>(b), static_cast<float>(c)} * h);
                    e[static_cast<std::size_t>(n)] = it->second;
                }
                elems.push_back(e);
                elemIdx.push_back({i, j, k});
            }
    const std::size_t N = nodePos.size(), ndof = 3 * N;
    if (elems.empty()) return fail("The body couldn't be filled with cubes (is it closed?)");
    // Surface nodes: on a face of a solid cube whose neighbour across it is empty.
    std::vector<char> surface(N, 0);
    static const int kFaces[6][4] = {{0, 2, 4, 6}, {1, 3, 5, 7}, {0, 1, 4, 5}, {2, 3, 6, 7}, {0, 1, 2, 3}, {4, 5, 6, 7}};
    static const int kStep[6][3] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};
    for (std::size_t e = 0; e < elems.size(); ++e)
        for (int f = 0; f < 6; ++f)
            if (!cellAt(elemIdx[e][0] + kStep[f][0], elemIdx[e][1] + kStep[f][1], elemIdx[e][2] + kStep[f][2]))
                for (int q = 0; q < 4; ++q) surface[static_cast<std::size_t>(elems[e][static_cast<std::size_t>(kFaces[f][q])])] = 1;
    // Selections.
    auto pick = [&](const FeaSelection& s) {
        std::vector<int> nodes;
        if (s.kind == FeaSelection::Kind::Near) {
            for (std::size_t n = 0; n < N; ++n)
                if (surface[n] && length(nodePos[n] - s.point) <= s.radius + 0.5f * h) nodes.push_back(static_cast<int>(n));
            return nodes;
        }
        // FACE: the triangle nearest the point gives the plane; surface nodes on it.
        float best = 1e30f;
        Vec3 q{}, nrm{0, 0, 1};
        for (std::size_t t = 0; t + 2 < world.indices.size(); t += 3) {
            const Vec3 a = world.vertices[world.indices[t]].position, b = world.vertices[world.indices[t + 1]].position,
                       c = world.vertices[world.indices[t + 2]].position;
            const Vec3 nn = cross(b - a, c - a);
            if (!(length(nn) > 0)) continue;
            // The point's distance to the triangle (via its plane and edges, roughly).
            const Vec3 nh = normalize(nn);
            const Vec3 onPlane = s.point - nh * dot(s.point - a, nh);
            const Vec3 c0 = cross(b - a, onPlane - a), c1 = cross(c - b, onPlane - b), c2 = cross(a - c, onPlane - c);
            const bool inside = dot(c0, nn) >= 0 && dot(c1, nn) >= 0 && dot(c2, nn) >= 0;
            const float d = inside ? std::fabs(dot(s.point - a, nh)) : std::min({length(s.point - a), length(s.point - b), length(s.point - c)});
            if (d < best) best = d, q = onPlane, nrm = nh;
        }
        for (std::size_t n = 0; n < N; ++n)
            if (surface[n] && std::fabs(dot(nodePos[n] - q, nrm)) <= 0.75f * h) nodes.push_back(static_cast<int>(n));
        return nodes;
    };
    std::vector<char> fixedDof(ndof, 0);
    std::vector<int> fixedNodes;
    for (const FeaSelection& s : setup.fixed) {
        const std::vector<int> nodes = pick(s);
        if (nodes.empty()) return fail("A FIX picks no surface nodes there");
        for (int n : nodes) {
            if (!fixedDof[static_cast<std::size_t>(3 * n)]) fixedNodes.push_back(n);
            for (int d = 0; d < 3; ++d) fixedDof[static_cast<std::size_t>(3 * n + d)] = 1;
        }
    }
    std::vector<double> f(ndof, 0.0);
    int loaded = 0;
    for (const FeaLoad& l : setup.loads) {
        const std::vector<int> nodes = pick(l.where);
        if (nodes.empty()) return fail("A LOAD picks no surface nodes there");
        for (int n : nodes)
            for (int d = 0; d < 3; ++d) f[static_cast<std::size_t>(3 * n + d)] += static_cast<double>((&l.force.x)[d]) / static_cast<double>(nodes.size());
        loaded += static_cast<int>(nodes.size());
    }
    for (std::size_t i = 0; i < ndof; ++i)
        if (fixedDof[i]) f[i] = 0;
    // K = E h Ke (every element the same cube).
    const std::array<double, 24 * 24> Ke = unitStiffness(mat.nu);
    const double scale = mat.E * static_cast<double>(h);
    auto applyK = [&](const std::vector<double>& x, std::vector<double>& y) {
        std::fill(y.begin(), y.end(), 0.0);
        double xe[24], ye[24];
        for (const auto& e : elems) {
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
        for (std::size_t i = 0; i < ndof; ++i)
            if (fixedDof[i]) y[i] = x[i]; // fixed: the identity
    };
    // Jacobi preconditioner: K's diagonal.
    std::vector<double> diag(ndof, 0.0);
    for (const auto& e : elems)
        for (int n = 0; n < 8; ++n)
            for (int d = 0; d < 3; ++d) diag[static_cast<std::size_t>(3 * e[static_cast<std::size_t>(n)] + d)] += Ke[static_cast<std::size_t>((3 * n + d) * 25)] * scale;
    for (std::size_t i = 0; i < ndof; ++i)
        if (fixedDof[i] || !(diag[i] > 0)) diag[i] = 1;
    std::vector<double> u(ndof, 0.0), r = f, z(ndof), p(ndof), Ap(ndof);
    double fn = 0;
    for (double v : f) fn += v * v;
    fn = std::sqrt(fn);
    if (!(fn > 0)) return fail("The loads add up to nothing");
    for (std::size_t i = 0; i < ndof; ++i) z[i] = r[i] / diag[i];
    p = z;
    double rz = 0;
    for (std::size_t i = 0; i < ndof; ++i) rz += r[i] * z[i];
    int it = 0;
    bool converged = false;
    for (; it < setup.maxIterations; ++it) {
        applyK(p, Ap);
        double pAp = 0;
        for (std::size_t i = 0; i < ndof; ++i) pAp += p[i] * Ap[i];
        if (!(pAp > 0)) return fail("The body isn't held firmly enough: it could move or turn freely (FIX more, or a whole face)");
        const double alpha = rz / pAp;
        double rr = 0;
        for (std::size_t i = 0; i < ndof; ++i) {
            u[i] += alpha * p[i];
            r[i] -= alpha * Ap[i];
            rr += r[i] * r[i];
        }
        if (std::sqrt(rr) <= setup.tolerance * fn) {
            converged = true;
            ++it;
            break;
        }
        double rz2 = 0;
        for (std::size_t i = 0; i < ndof; ++i) z[i] = r[i] / diag[i], rz2 += r[i] * z[i];
        const double beta = rz2 / rz;
        rz = rz2;
        for (std::size_t i = 0; i < ndof; ++i) p[i] = z[i] + beta * p[i];
    }
    // A body free to move shows as huge displacements.
    double umax = 0;
    std::size_t umaxAt = 0;
    for (std::size_t n = 0; n < N; ++n) {
        const double m = std::sqrt(u[3 * n] * u[3 * n] + u[3 * n + 1] * u[3 * n + 1] + u[3 * n + 2] * u[3 * n + 2]);
        if (m > umax) umax = m, umaxAt = n;
    }
    if (!std::isfinite(umax) || umax > 1e3 * static_cast<double>(longest)) return fail("The body isn't held firmly enough: it could move or turn freely");
    // Stress at each cube's corners (D B u there, then von Mises), averaged over the cubes
    // meeting at a node: corners lie on the surface, where bending stress peaks.
    const double lam = mat.E * mat.nu / ((1 + mat.nu) * (1 - 2 * mat.nu)), mu = mat.E / (2 * (1 + mat.nu));
    std::vector<double> nodeStress(N, 0.0);
    std::vector<int> nodeCount(N, 0);
    for (std::size_t e = 0; e < elems.size(); ++e)
        for (int c = 0; c < 8; ++c) {
            const double xi[3] = {static_cast<double>(c & 1), static_cast<double>((c >> 1) & 1), static_cast<double>((c >> 2) & 1)};
            double eps[6] = {};
            for (int n = 0; n < 8; ++n) {
                const double sgn[3] = {static_cast<double>(n & 1), static_cast<double>((n >> 1) & 1), static_cast<double>((n >> 2) & 1)};
                double fc[3], dN[3];
                for (int k = 0; k < 3; ++k) fc[k] = sgn[k] ? xi[k] : 1 - xi[k];
                for (int k = 0; k < 3; ++k) {
                    double d = (sgn[k] ? 1.0 : -1.0) / static_cast<double>(h);
                    for (int m = 0; m < 3; ++m)
                        if (m != k) d *= fc[m];
                    dN[k] = d;
                }
                const std::size_t id = static_cast<std::size_t>(elems[e][static_cast<std::size_t>(n)]);
                const double ux = u[3 * id], uy = u[3 * id + 1], uz = u[3 * id + 2];
                eps[0] += dN[0] * ux, eps[1] += dN[1] * uy, eps[2] += dN[2] * uz;
                eps[3] += dN[1] * ux + dN[0] * uy, eps[4] += dN[2] * uy + dN[1] * uz, eps[5] += dN[2] * ux + dN[0] * uz;
            }
            const double tr = eps[0] + eps[1] + eps[2];
            const double sx = lam * tr + 2 * mu * eps[0], sy = lam * tr + 2 * mu * eps[1], sz = lam * tr + 2 * mu * eps[2];
            const double txy = mu * eps[3], tyz = mu * eps[4], txz = mu * eps[5];
            const double vm = std::sqrt(0.5 * ((sx - sy) * (sx - sy) + (sy - sz) * (sy - sz) + (sz - sx) * (sz - sx)) + 3 * (txy * txy + tyz * tyz + txz * txz));
            const std::size_t node = static_cast<std::size_t>(elems[e][static_cast<std::size_t>(c)]);
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
    out.cell = h;
    out.cells = static_cast<int>(elems.size());
    out.nodes = static_cast<int>(N);
    out.fixedNodes = static_cast<int>(fixedNodes.size());
    out.loadedNodes = loaded;
    out.iterations = it;
    out.converged = converged;
    out.maxStress = smax;
    out.maxStressAt = nodePos[smaxAt];
    out.maxDisplacement = umax;
    out.maxDisplacementAt = nodePos[umaxAt];
    out.safetyFactor = out.maxStress > 0 ? mat.yield / out.maxStress : 0;
    // The surface's values: trilinear between the corners of the cube the point is in (those
    // corners that are nodes), else the nearest node about.
    out.surfaceStress.resize(world.vertices.size());
    out.surfaceDisplacement.resize(world.vertices.size());
    auto nodeAt = [&](long long a, long long b, long long c) -> int {
        const auto itn = nodeOf.find((c * NY + b) * NX + a);
        return itn == nodeOf.end() ? -1 : itn->second;
    };
    for (std::size_t v = 0; v < world.vertices.size(); ++v) {
        const Vec3 g = (world.vertices[v].position - origin) / h;
        const long long ci = std::clamp<long long>(static_cast<long long>(std::floor(g.x)), 0, nx - 1);
        const long long cj = std::clamp<long long>(static_cast<long long>(std::floor(g.y)), 0, ny - 1);
        const long long ck = std::clamp<long long>(static_cast<long long>(std::floor(g.z)), 0, nz - 1);
        const float fx = std::clamp(g.x - static_cast<float>(ci), 0.0f, 1.0f), fy = std::clamp(g.y - static_cast<float>(cj), 0.0f, 1.0f),
                    fz = std::clamp(g.z - static_cast<float>(ck), 0.0f, 1.0f);
        double wsum = 0, sv = 0;
        Vec3 dv{};
        for (int n = 0; n < 8; ++n) {
            const int id = nodeAt(ci + (n & 1), cj + ((n >> 1) & 1), ck + ((n >> 2) & 1));
            if (id < 0) continue;
            const double w = static_cast<double>((n & 1) ? fx : 1 - fx) * ((n & 2) ? fy : 1 - fy) * ((n & 4) ? fz : 1 - fz) + 1e-9;
            const std::size_t b = static_cast<std::size_t>(id);
            wsum += w, sv += w * nodeStress[b];
            dv = dv + Vec3{static_cast<float>(u[3 * b]), static_cast<float>(u[3 * b + 1]), static_cast<float>(u[3 * b + 2])} * static_cast<float>(w);
        }
        if (wsum <= 0) { // outside the filled cubes: the nearest node near by
            float bestD = 1e30f;
            for (int dk = -1; dk <= 2; ++dk)
                for (int dj = -1; dj <= 2; ++dj)
                    for (int di = -1; di <= 2; ++di) {
                        const int id = nodeAt(ci + di, cj + dj, ck + dk);
                        if (id < 0) continue;
                        const float d = length(nodePos[static_cast<std::size_t>(id)] - world.vertices[v].position);
                        if (d < bestD) {
                            bestD = d;
                            const std::size_t b = static_cast<std::size_t>(id);
                            wsum = 1, sv = nodeStress[b];
                            dv = {static_cast<float>(u[3 * b]), static_cast<float>(u[3 * b + 1]), static_cast<float>(u[3 * b + 2])};
                        }
                    }
            if (wsum <= 0) continue;
        }
        out.surfaceStress[v] = static_cast<float>(sv / wsum);
        out.surfaceDisplacement[v] = dv / static_cast<float>(wsum);
    }
    return true;
}

} // namespace sicdm

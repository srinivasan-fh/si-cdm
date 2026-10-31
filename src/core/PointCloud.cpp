#include "core/PointCloud.hpp"

#include "core/Armature.hpp"
#include "core/Sculpt.hpp"
#include "core/Wearable.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <queue>
#include <sstream>

namespace sicdm {

namespace {

bool failWith(std::string* why, const std::string& w) {
    if (why) *why = w;
    return false;
}

std::string lowerExt(const std::string& path) {
    const std::size_t dot = path.find_last_of('.');
    std::string e = dot == std::string::npos ? "" : path.substr(dot + 1);
    for (char& c : e) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return e;
}

// Numbers on a line, split by spaces, tabs, commas or semicolons.
std::vector<double> numbersOf(const std::string& line) {
    std::vector<double> out;
    const char* p = line.c_str();
    while (*p) {
        while (*p == ' ' || *p == '\t' || *p == ',' || *p == ';' || *p == '\r') ++p;
        if (!*p) break;
        char* end = nullptr;
        const double v = std::strtod(p, &end);
        if (end == p) return {}; // a word: a header line, not a point
        out.push_back(v);
        p = end;
    }
    return out;
}

bool readPly(std::ifstream& in, std::vector<Vec3>& pts, std::vector<Vec3>& nrm, std::string* why) {
    std::string line, format;
    struct Prop {
        std::string name, type;
        bool list = false;
        std::string countType;
    };
    struct Element {
        std::string name;
        std::size_t count = 0;
        std::vector<Prop> props;
    };
    std::vector<Element> els;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::istringstream ls(line);
        std::string w;
        ls >> w;
        if (w == "format") ls >> format;
        else if (w == "element") {
            Element e;
            ls >> e.name >> e.count;
            els.push_back(e);
        } else if (w == "property" && !els.empty()) {
            Prop p;
            ls >> p.type;
            if (p.type == "list") p.list = true, ls >> p.countType >> p.type;
            ls >> p.name;
            els.back().props.push_back(p);
        } else if (w == "end_header") break;
    }
    if (format != "ascii" && format != "binary_little_endian") return failWith(why, "PLY format " + format + " is not read (ASCII or little-endian binary)");
    auto size = [](const std::string& t) -> int {
        if (t == "char" || t == "uchar" || t == "int8" || t == "uint8") return 1;
        if (t == "short" || t == "ushort" || t == "int16" || t == "uint16") return 2;
        if (t == "int" || t == "uint" || t == "float" || t == "int32" || t == "uint32" || t == "float32") return 4;
        if (t == "double" || t == "float64") return 8;
        return 0;
    };
    auto readBinary = [&](const std::string& t) -> double {
        unsigned char b[8] = {};
        in.read(reinterpret_cast<char*>(b), size(t));
        if (t == "float" || t == "float32") {
            float f;
            std::memcpy(&f, b, 4);
            return f;
        }
        if (t == "double" || t == "float64") {
            double d;
            std::memcpy(&d, b, 8);
            return d;
        }
        if (t == "char" || t == "int8") return static_cast<signed char>(b[0]);
        if (t == "uchar" || t == "uint8") return b[0];
        if (t == "short" || t == "int16") return static_cast<std::int16_t>(b[0] | b[1] << 8);
        if (t == "ushort" || t == "uint16") return static_cast<std::uint16_t>(b[0] | b[1] << 8);
        std::uint32_t u = static_cast<std::uint32_t>(b[0]) | static_cast<std::uint32_t>(b[1]) << 8 | static_cast<std::uint32_t>(b[2]) << 16 |
                          static_cast<std::uint32_t>(b[3]) << 24;
        return t == "uint" || t == "uint32" ? static_cast<double>(u) : static_cast<double>(static_cast<std::int32_t>(u));
    };
    const bool binary = format != "ascii";
    for (const Element& e : els) {
        int ix = -1, iy = -1, iz = -1, inx = -1, iny = -1, inz = -1;
        for (std::size_t k = 0; k < e.props.size(); ++k) {
            const std::string& n = e.props[k].name;
            const int i = static_cast<int>(k);
            if (n == "x") ix = i;
            else if (n == "y") iy = i;
            else if (n == "z") iz = i;
            else if (n == "nx") inx = i;
            else if (n == "ny") iny = i;
            else if (n == "nz") inz = i;
        }
        const bool vertex = e.name == "vertex" && ix >= 0 && iy >= 0 && iz >= 0;
        const bool normals = vertex && inx >= 0 && iny >= 0 && inz >= 0;
        std::vector<double> v(e.props.size());
        for (std::size_t r = 0; r < e.count; ++r) {
            if (binary) {
                for (std::size_t k = 0; k < e.props.size(); ++k) {
                    const Prop& p = e.props[k];
                    if (!size(p.type)) return failWith(why, "PLY property type " + p.type + " is not read");
                    if (p.list) {
                        const auto n = static_cast<std::size_t>(readBinary(p.countType));
                        for (std::size_t q = 0; q < n; ++q) readBinary(p.type);
                        v[k] = 0;
                    } else v[k] = readBinary(p.type);
                }
                if (!in) return failWith(why, "the PLY file ends early");
            } else {
                if (!std::getline(in, line)) return failWith(why, "the PLY file ends early");
                if (vertex) {
                    const std::vector<double> nums = numbersOf(line);
                    if (nums.size() < e.props.size()) return failWith(why, "a PLY vertex line is short");
                    for (std::size_t k = 0; k < e.props.size(); ++k) v[k] = nums[k];
                }
            }
            if (vertex) {
                pts.push_back({static_cast<float>(v[static_cast<std::size_t>(ix)]), static_cast<float>(v[static_cast<std::size_t>(iy)]),
                               static_cast<float>(v[static_cast<std::size_t>(iz)])});
                if (normals)
                    nrm.push_back({static_cast<float>(v[static_cast<std::size_t>(inx)]), static_cast<float>(v[static_cast<std::size_t>(iny)]),
                                   static_cast<float>(v[static_cast<std::size_t>(inz)])});
            }
        }
    }
    return true;
}

// The eigenvector of a symmetric 3 x 3 matrix with the smallest eigenvalue (Jacobi).
Vec3 smallestAxis(double a[3][3]) {
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int sweep = 0; sweep < 30; ++sweep) {
        double off = std::fabs(a[0][1]) + std::fabs(a[0][2]) + std::fabs(a[1][2]);
        if (off < 1e-18) break;
        for (int p = 0; p < 2; ++p)
            for (int q = p + 1; q < 3; ++q) {
                if (std::fabs(a[p][q]) < 1e-30) continue;
                const double th = 0.5 * std::atan2(2 * a[p][q], a[q][q] - a[p][p]), c = std::cos(th), s = std::sin(th);
                for (int k = 0; k < 3; ++k) { // A <- J^T A J
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq, a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk, a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq, v[k][q] = s * vkp + c * vkq;
                }
            }
    }
    int m = 0;
    for (int k = 1; k < 3; ++k)
        if (a[k][k] < a[m][m]) m = k;
    return normalize(Vec3{static_cast<float>(v[0][m]), static_cast<float>(v[1][m]), static_cast<float>(v[2][m])});
}

// Each point's k nearest (itself left out), by growing the search until there are enough.
std::vector<std::vector<std::uint32_t>> nearestK(const std::vector<Vec3>& pts, int k, float spacing) {
    NearestPoints near(pts, std::max(spacing * 2.0f, 1e-6f));
    std::vector<std::vector<std::uint32_t>> out(pts.size());
    std::vector<std::uint32_t> found;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        float r = std::max(spacing * 2.0f, 1e-6f);
        for (int tries = 0; tries < 12; ++tries, r *= 2.0f) {
            found.clear();
            near.around(pts[i], r, found);
            if (static_cast<int>(found.size()) > k || found.size() == pts.size()) break;
        }
        std::sort(found.begin(), found.end(), [&](std::uint32_t a, std::uint32_t b) { return length(pts[a] - pts[i]) < length(pts[b] - pts[i]); });
        for (std::uint32_t j : found)
            if (j != i && static_cast<int>(out[i].size()) < k) out[i].push_back(j);
    }
    return out;
}

} // namespace

bool readPointCloud(const std::string& path, std::vector<Vec3>& pts, std::vector<Vec3>& nrm, std::string* why) {
    pts.clear(), nrm.clear();
    std::ifstream in(path, std::ios::binary);
    if (!in) return failWith(why, "Cannot open " + path);
    const std::string ext = lowerExt(path);
    if (ext == "ply") {
        std::string magic;
        std::getline(in, magic);
        if (magic.rfind("ply", 0) != 0) return failWith(why, path + " is not a PLY file");
        if (!readPly(in, pts, nrm, why)) return false;
    } else {
        std::string line;
        bool withNormals = true;
        while (std::getline(in, line)) {
            const std::vector<double> v = numbersOf(line);
            if (v.size() < 3) continue; // a count or a header
            pts.push_back({static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])});
            // Six numbers whose last three make a unit vector: normals (else colours, intensity...).
            const bool n = v.size() == 6 && std::fabs(std::sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5]) - 1.0) < 0.05;
            withNormals = withNormals && n;
            if (withNormals) nrm.push_back(normalize(Vec3{static_cast<float>(v[3]), static_cast<float>(v[4]), static_cast<float>(v[5])}));
        }
        if (!withNormals) nrm.clear();
    }
    if (nrm.size() != pts.size()) nrm.clear();
    if (pts.size() < 4) return failWith(why, path + " has fewer than four points");
    return true;
}

float pointSpacing(const std::vector<Vec3>& pts) {
    if (pts.size() < 2) return 0;
    AABB box;
    for (const Vec3& p : pts) box.expand(p);
    const Vec3 ext = box.max - box.min;
    // A first guess from the volume per point, then the median nearest distance of a sample.
    const float guess = std::max(1e-6f, std::cbrt(std::max(ext.x, 1e-3f) * std::max(ext.y, 1e-3f) * std::max(ext.z, 1e-3f) / static_cast<float>(pts.size())));
    NearestPoints near(pts, guess * 2.0f);
    std::vector<float> d;
    const std::size_t step = std::max<std::size_t>(1, pts.size() / 2000);
    std::vector<std::uint32_t> found;
    for (std::size_t i = 0; i < pts.size(); i += step) {
        float r = guess * 2.0f, best = 0;
        for (int tries = 0; tries < 16 && best == 0; ++tries, r *= 2.0f) {
            found.clear();
            near.around(pts[i], r, found);
            float b = 1e30f;
            for (std::uint32_t j : found)
                if (j != i) b = std::min(b, length(pts[j] - pts[i]));
            if (b < 1e30f) best = b;
        }
        if (best > 0) d.push_back(best);
    }
    if (d.empty()) return guess;
    std::nth_element(d.begin(), d.begin() + static_cast<std::ptrdiff_t>(d.size() / 2), d.end());
    return d[d.size() / 2];
}

std::vector<Vec3> estimateNormals(const std::vector<Vec3>& pts, int k) {
    std::vector<Vec3> n(pts.size(), Vec3{0, 0, 1});
    if (pts.size() < 3) return n;
    const auto nbr = nearestK(pts, k, pointSpacing(pts));
    for (std::size_t i = 0; i < pts.size(); ++i) {
        Vec3 c = pts[i];
        for (std::uint32_t j : nbr[i]) c = c + pts[j];
        c = c * (1.0f / static_cast<float>(nbr[i].size() + 1));
        double a[3][3] = {};
        auto add = [&](const Vec3& p) {
            const double d[3] = {p.x - c.x, p.y - c.y, p.z - c.z};
            for (int r = 0; r < 3; ++r)
                for (int s = 0; s < 3; ++s) a[r][s] += d[r] * d[s];
        };
        add(pts[i]);
        for (std::uint32_t j : nbr[i]) add(pts[j]);
        n[i] = smallestAxis(a);
    }
    // Turned to agree: from the highest point (facing up), each next the neighbour across the
    // flattest join (its normal nearest parallel), as a minimum spanning tree grows.
    std::vector<char> done(pts.size(), 0);
    for (;;) {
        std::size_t seed = pts.size();
        for (std::size_t i = 0; i < pts.size(); ++i)
            if (!done[i] && (seed == pts.size() || pts[i].z > pts[seed].z)) seed = i;
        if (seed == pts.size()) break;
        if (n[seed].z < 0) n[seed] = n[seed] * -1.0f;
        using Item = std::pair<float, std::pair<std::uint32_t, std::uint32_t>>; // cost, (from, to)
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
        auto push = [&](std::uint32_t from) {
            for (std::uint32_t to : nbr[from])
                if (!done[to]) q.push({1.0f - std::fabs(dot(n[from], n[to])), {from, to}});
        };
        done[seed] = 1;
        push(static_cast<std::uint32_t>(seed));
        while (!q.empty()) {
            const auto [cost, e] = q.top();
            q.pop();
            const std::uint32_t to = e.second;
            if (done[to]) continue;
            if (dot(n[e.first], n[to]) < 0) n[to] = n[to] * -1.0f;
            done[to] = 1;
            push(to);
        }
    }
    return n;
}

MeshData reconstructSurface(const std::vector<Vec3>& pts, const std::vector<Vec3>& nrmIn, float voxel, ReconstructReport* report, std::string* why) {
    MeshData out;
    if (pts.size() < 10) return failWith(why, "too few points (at least 10)"), out;
    const std::vector<Vec3> nrm = nrmIn.size() == pts.size() ? nrmIn : estimateNormals(pts);
    const float spacing = pointSpacing(pts);
    AABB box;
    for (const Vec3& p : pts) box.expand(p);
    const Vec3 ext = box.max - box.min;
    float h = voxel > 0 ? voxel : spacing * 2.0f;
    const float longest = std::max({ext.x, ext.y, ext.z});
    h = std::max(h, longest / 220.0f); // at most about 220 cells along the longest side
    const float reach = std::max(3.0f * h, 3.0f * spacing);
    const Vec3 origin = box.min - Vec3{reach, reach, reach};
    const int nx = static_cast<int>(std::ceil((ext.x + 2 * reach) / h)) + 1, ny = static_cast<int>(std::ceil((ext.y + 2 * reach) / h)) + 1,
              nz = static_cast<int>(std::ceil((ext.z + 2 * reach) / h)) + 1;
    NearestPoints near(pts, std::max(h, spacing));
    // Signed distance to the nearest point's tangent plane (Hoppe), averaged over the few
    // nearest for a smoother surface; far from every point, outside.
    std::vector<float> f(static_cast<std::size_t>(nx) * ny * nz);
    std::vector<char> far(f.size(), 0); // no point within reach: inside or outside, found below
    std::vector<std::uint32_t> found;
    for (int x = 0; x < nx; ++x)
        for (int y = 0; y < ny; ++y)
            for (int z = 0; z < nz; ++z) {
                const Vec3 c = origin + Vec3{(static_cast<float>(x) + 0.5f) * h, (static_cast<float>(y) + 0.5f) * h, (static_cast<float>(z) + 0.5f) * h};
                found.clear();
                near.around(c, reach, found);
                float value = reach;
                if (found.empty()) far[(static_cast<std::size_t>(x) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(y)) * static_cast<std::size_t>(nz) +
                                       static_cast<std::size_t>(z)] = 1;
                else {
                    // The nearest few, weighted by nearness.
                    std::partial_sort(found.begin(), found.begin() + static_cast<std::ptrdiff_t>(std::min<std::size_t>(6, found.size())), found.end(),
                                      [&](std::uint32_t a, std::uint32_t b) { return length(pts[a] - c) < length(pts[b] - c); });
                    double sum = 0, wsum = 0;
                    for (std::size_t q = 0; q < std::min<std::size_t>(6, found.size()); ++q) {
                        const std::uint32_t j = found[q];
                        const float d = length(pts[j] - c);
                        const double w = 1.0 / (d * d + spacing * spacing);
                        sum += w * dot(c - pts[j], nrm[j]), wsum += w;
                    }
                    value = static_cast<float>(sum / wsum);
                }
                f[(static_cast<std::size_t>(x) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(y)) * static_cast<std::size_t>(nz) + static_cast<std::size_t>(z)] =
                    value;
            }
    // Far cells joined to the grid's sides through far cells are outside; the rest (a closed
    // scan's hollow middle) inside.
    {
        auto id = [&](int x, int y, int z) {
            return (static_cast<std::size_t>(x) * static_cast<std::size_t>(ny) + static_cast<std::size_t>(y)) * static_cast<std::size_t>(nz) + static_cast<std::size_t>(z);
        };
        std::vector<char> outside(f.size(), 0);
        std::vector<std::array<int, 3>> stack;
        for (int x = 0; x < nx; ++x)
            for (int y = 0; y < ny; ++y)
                for (int z = 0; z < nz; ++z)
                    if ((x == 0 || y == 0 || z == 0 || x == nx - 1 || y == ny - 1 || z == nz - 1) && far[id(x, y, z)] && !outside[id(x, y, z)]) {
                        outside[id(x, y, z)] = 1;
                        stack.push_back({x, y, z});
                    }
        while (!stack.empty()) {
            const auto [x, y, z] = stack.back();
            stack.pop_back();
            const int d[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
            for (const auto& s3 : d) {
                const int a = x + s3[0], b = y + s3[1], c = z + s3[2];
                if (a < 0 || b < 0 || c < 0 || a >= nx || b >= ny || c >= nz) continue;
                const std::size_t k = id(a, b, c);
                if (far[k] && !outside[k]) outside[k] = 1, stack.push_back({a, b, c});
            }
        }
        for (std::size_t k = 0; k < f.size(); ++k)
            if (far[k] && !outside[k]) f[k] = -reach;
    }
    MeshData m = marchTets(f, nx, ny, nz, origin, h, true);
    // Triangles far from every point (where the field closed a gap the scan left) go.
    std::size_t dropped = 0;
    std::vector<std::uint32_t> keep;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3 c = (m.vertices[m.indices[t]].position + m.vertices[m.indices[t + 1]].position + m.vertices[m.indices[t + 2]].position) * (1.0f / 3.0f);
        if (near.find(c, std::max(3.0f * h, 4.0f * spacing)) < 0) {
            ++dropped;
            continue;
        }
        keep.insert(keep.end(), {m.indices[t], m.indices[t + 1], m.indices[t + 2]});
    }
    m.indices = std::move(keep);
    if (m.indices.empty()) return failWith(why, "no surface came out (try a larger VOXEL)"), out;
    simplifyMesh(m, h * 0.9f); // marching leaves slivers, and more triangles than the voxels hold
    smoothVertexNormals(m);
    m.computeFeatureEdges(45.0f);
    if (report) report->voxel = h, report->triangles = m.indices.size() / 3, report->dropped = dropped;
    return m;
}

} // namespace sicdm

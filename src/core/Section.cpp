#include "core/Section.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace sicdm::section {

namespace {

std::uint64_t edgeOf(std::uint32_t a, std::uint32_t b) {
    return a < b ? (static_cast<std::uint64_t>(a) << 32) | b : (static_cast<std::uint64_t>(b) << 32) | a;
}

Vec3 faceNormal(const std::vector<Vec3>& p, const Tri& t) { return cross(p[t[1]] - p[t[0]], p[t[2]] - p[t[0]]); }

} // namespace

double loopArea(const Loop& l) {
    double a = 0;
    for (std::size_t i = 0, n = l.size(); i < n; ++i) a += l[i].x * l[(i + 1) % n].y - l[(i + 1) % n].x * l[i].y;
    return 0.5 * a;
}

std::vector<Loop> offsetLeft(const std::vector<Loop>& loops, double d) {
    std::vector<Loop> out;
    for (const Loop& l : loops) {
        const std::size_t n = l.size();
        if (n < 3) continue;
        Loop o(n);
        for (std::size_t i = 0; i < n; ++i) {
            const P2 &a = l[(i + n - 1) % n], &b = l[i], &c = l[(i + 1) % n];
            auto leftOf = [](const P2& u, const P2& v) {
                const double dx = v.x - u.x, dy = v.y - u.y, len = std::hypot(dx, dy);
                return len > 0 ? P2{-dy / len, dx / len} : P2{0, 0};
            };
            const P2 n1 = leftOf(a, b), n2 = leftOf(b, c);
            const double k = 1.0 + n1.x * n2.x + n1.y * n2.y;
            const double s = k > 0.25 ? d / k : d / 0.25; // a mitre, clamped at sharp corners
            o[i] = {b.x + (n1.x + n2.x) * s, b.y + (n1.y + n2.y) * s};
        }
        const double a0 = loopArea(l), a1 = loopArea(o);
        if (a0 * a1 <= 0 || std::fabs(a1) < d * d) continue;
        out.push_back(std::move(o));
    }
    return out;
}

std::vector<Loop> contours(const std::vector<Vec3>& p, const std::vector<Tri>& tris, const std::vector<std::uint32_t>& candidates,
                           double z) {
    struct Seg {
        std::uint64_t from, to;
        P2 a, b;
    };
    std::vector<Seg> segs;
    auto cut = [&](std::uint32_t u, std::uint32_t v) {
        const double t = (z - p[u].z) / (static_cast<double>(p[v].z) - p[u].z);
        return P2{p[u].x + (static_cast<double>(p[v].x) - p[u].x) * t, p[u].y + (static_cast<double>(p[v].y) - p[u].y) * t};
    };
    for (const std::uint32_t ti : candidates) {
        const Tri& t = tris[ti];
        std::uint32_t ends[2][2];
        int k = 0;
        for (int e = 0; e < 3 && k < 2; ++e) {
            const std::uint32_t u = t[static_cast<std::size_t>(e)], v = t[static_cast<std::size_t>((e + 1) % 3)];
            if ((p[u].z < z) != (p[v].z < z)) {
                ends[k][0] = u;
                ends[k][1] = v;
                ++k;
            }
        }
        if (k != 2) continue;
        Seg s{edgeOf(ends[0][0], ends[0][1]), edgeOf(ends[1][0], ends[1][1]), cut(ends[0][0], ends[0][1]), cut(ends[1][0], ends[1][1])};
        // Material on the left: the segment runs along (-ny, nx) of the outward normal.
        const Vec3 n = faceNormal(p, t);
        const double dx = s.b.x - s.a.x, dy = s.b.y - s.a.y;
        if (dx * -n.y + dy * n.x < 0) {
            std::swap(s.from, s.to);
            std::swap(s.a, s.b);
        }
        segs.push_back(s);
    }
    std::unordered_map<std::uint64_t, std::size_t> starting;
    starting.reserve(segs.size() * 2);
    for (std::size_t i = 0; i < segs.size(); ++i) starting.emplace(segs[i].from, i);
    std::vector<char> used(segs.size(), 0);
    std::vector<Loop> loops;
    for (std::size_t i = 0; i < segs.size(); ++i) {
        if (used[i]) continue;
        Loop l;
        std::size_t j = i;
        bool closed = false;
        for (std::size_t guard = 0; guard <= segs.size(); ++guard) {
            used[j] = 1;
            l.push_back(segs[j].a);
            const auto it = starting.find(segs[j].to);
            if (it == starting.end()) break;
            if (it->second == i) {
                closed = true;
                break;
            }
            if (used[it->second]) break;
            j = it->second;
        }
        if (!closed) continue;
        // Points a hair apart (where a contour crosses a corner of the mesh) and points
        // in a straight line would give the offset walls spikes.
        Loop clean;
        for (const P2& q : l)
            if (clean.empty() || std::hypot(q.x - clean.back().x, q.y - clean.back().y) > 1e-4) clean.push_back(q);
        while (clean.size() > 1 && std::hypot(clean.front().x - clean.back().x, clean.front().y - clean.back().y) <= 1e-4) clean.pop_back();
        for (bool again = true; again && clean.size() >= 3;) {
            again = false;
            for (std::size_t k = 0; k < clean.size() && clean.size() >= 3; ++k) {
                const P2 &a = clean[(k + clean.size() - 1) % clean.size()], &b = clean[k], &c = clean[(k + 1) % clean.size()];
                const double ux = b.x - a.x, uy = b.y - a.y, vx = c.x - b.x, vy = c.y - b.y;
                const double turn = ux * vy - uy * vx, run = std::hypot(ux, uy) * std::hypot(vx, vy);
                if (std::fabs(turn) <= run * 1e-6 && ux * vx + uy * vy > 0) {
                    clean.erase(clean.begin() + static_cast<std::ptrdiff_t>(k));
                    again = true;
                }
            }
        }
        if (clean.size() >= 3) loops.push_back(std::move(clean));
    }
    return loops;
}

std::vector<std::pair<P2, P2>> hatch(const std::vector<Loop>& loops, double spacing, double angle) {
    std::vector<std::pair<P2, P2>> out;
    if (loops.empty() || !(spacing > 0)) return out;
    const double c = std::cos(angle), s = std::sin(angle);
    auto to = [&](const P2& q) { return P2{q.x * c + q.y * s, -q.x * s + q.y * c}; };
    auto back = [&](const P2& q) { return P2{q.x * c - q.y * s, q.x * s + q.y * c}; };
    std::vector<Loop> r;
    double lo = 1e300, hi = -1e300;
    for (const Loop& l : loops) {
        Loop t;
        for (const P2& q : l) {
            t.push_back(to(q));
            lo = std::min(lo, t.back().y);
            hi = std::max(hi, t.back().y);
        }
        r.push_back(std::move(t));
    }
    std::vector<double> xs;
    bool flip = false;
    for (double y = std::floor(lo / spacing) * spacing + spacing * 0.5; y < hi; y += spacing) {
        xs.clear();
        for (const Loop& l : r)
            for (std::size_t i = 0, n = l.size(); i < n; ++i) {
                const P2 &a = l[i], &b = l[(i + 1) % n];
                if ((a.y <= y) == (b.y <= y)) continue;
                xs.push_back(a.x + (b.x - a.x) * (y - a.y) / (b.y - a.y));
            }
        std::sort(xs.begin(), xs.end());
        for (std::size_t i = 0; i + 1 < xs.size(); i += 2) {
            if (xs[i + 1] - xs[i] < spacing * 0.1) continue;
            P2 a = back({xs[i], y}), b = back({xs[i + 1], y});
            if (flip) std::swap(a, b);
            out.push_back({a, b});
        }
        flip = !flip;
    }
    return out;
}

} // namespace sicdm::section

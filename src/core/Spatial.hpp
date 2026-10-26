#pragma once

// Small fixed-size algebra in double precision for the robot engines: 3-vectors, 3 x 3
// matrices (row major), and Featherstone's spatial vectors and inertias with everything in
// the world frame (Plucker coordinates at the world origin; angular part first).

#include <array>
#include <cmath>
#include <utility>

namespace sicdm::spatial {

using V3d = std::array<double, 3>;
using M3 = std::array<double, 9>; // row major

inline V3d add(const V3d& a, const V3d& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
inline V3d sub(const V3d& a, const V3d& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
inline V3d mul(const V3d& a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
inline double dot(const V3d& a, const V3d& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline V3d cross(const V3d& a, const V3d& b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
inline double norm(const V3d& a) { return std::sqrt(dot(a, a)); }

inline V3d mv(const M3& m, const V3d& v) {
    return {m[0] * v[0] + m[1] * v[1] + m[2] * v[2], m[3] * v[0] + m[4] * v[1] + m[5] * v[2], m[6] * v[0] + m[7] * v[1] + m[8] * v[2]};
}
inline M3 mm(const M3& a, const M3& b) {
    M3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
    return r;
}
inline M3 transpose(const M3& a) { return {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]}; }
inline M3 identity3() { return {1, 0, 0, 0, 1, 0, 0, 0, 1}; }

// Rotation by `angle` about the unit `axis` (Rodrigues).
inline M3 rotation(const V3d& axis, double angle) {
    const double c = std::cos(angle), s = std::sin(angle), k = 1 - c;
    const double x = axis[0], y = axis[1], z = axis[2];
    return {c + x * x * k, x * y * k - z * s, x * z * k + y * s, y * x * k + z * s, c + y * y * k, y * z * k - x * s, z * x * k - y * s, z * y * k + x * s, c + z * z * k};
}

// Spatial motion / force vector: angular (or moment) part first.
struct SV {
    V3d w{0, 0, 0}, v{0, 0, 0};
};
inline SV operator+(const SV& a, const SV& b) { return {add(a.w, b.w), add(a.v, b.v)}; }
inline SV operator-(const SV& a, const SV& b) { return {sub(a.w, b.w), sub(a.v, b.v)}; }
inline SV operator*(const SV& a, double s) { return {mul(a.w, s), mul(a.v, s)}; }
inline double sdot(const SV& a, const SV& b) { return dot(a.w, b.w) + dot(a.v, b.v); }
// Motion cross motion: v x m u.
inline SV crm(const SV& v, const SV& u) { return {cross(v.w, u.w), add(cross(v.w, u.v), cross(v.v, u.w))}; }
// Motion cross force: v x* f.
inline SV crf(const SV& v, const SV& f) { return {add(cross(v.w, f.w), cross(v.v, f.v)), cross(v.w, f.v)}; }

// 6 x 6, row major, acting on (w, v).
struct M6 {
    double a[36] = {};
    SV operator*(const SV& x) const {
        const double in[6] = {x.w[0], x.w[1], x.w[2], x.v[0], x.v[1], x.v[2]};
        double o[6];
        for (int i = 0; i < 6; ++i) {
            double s = 0;
            for (int j = 0; j < 6; ++j) s += a[i * 6 + j] * in[j];
            o[i] = s;
        }
        return {{o[0], o[1], o[2]}, {o[3], o[4], o[5]}};
    }
    M6& operator+=(const M6& b) {
        for (int i = 0; i < 36; ++i) a[i] += b.a[i];
        return *this;
    }
};

// Spatial inertia at the world origin of mass m, centre c and inertia Ic about c.
inline M6 spatialInertia(double m, const V3d& c, const M3& Ic) {
    // [[Ic + m cx cx^T, m cx], [m cx^T, m 1]], cx = skew(c).
    const M3 cx = {0, -c[2], c[1], c[2], 0, -c[0], -c[1], c[0], 0};
    const M3 cxcxT = mm(cx, transpose(cx));
    M6 I;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            I.a[i * 6 + j] = Ic[i * 3 + j] + m * cxcxT[i * 3 + j];
            I.a[i * 6 + 3 + j] = m * cx[i * 3 + j];
            I.a[(3 + i) * 6 + j] = m * cx[j * 3 + i];
            I.a[(3 + i) * 6 + 3 + j] = i == j ? m : 0.0;
        }
    return I;
}

// Solves A x = b (6 x 6) by Gaussian elimination with partial pivoting.
inline SV solve6(const M6& A, const SV& b) {
    double m[6][7];
    const double rhs[6] = {b.w[0], b.w[1], b.w[2], b.v[0], b.v[1], b.v[2]};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) m[i][j] = A.a[i * 6 + j];
        m[i][6] = rhs[i];
    }
    for (int c = 0; c < 6; ++c) {
        int p = c;
        for (int r = c + 1; r < 6; ++r)
            if (std::fabs(m[r][c]) > std::fabs(m[p][c])) p = r;
        if (p != c)
            for (int k = 0; k < 7; ++k) std::swap(m[p][k], m[c][k]);
        const double d = std::fabs(m[c][c]) > 1e-300 ? m[c][c] : 1e-300;
        for (int r = c + 1; r < 6; ++r) {
            const double f = m[r][c] / d;
            if (f == 0) continue;
            for (int k = c; k < 7; ++k) m[r][k] -= f * m[c][k];
        }
    }
    double x[6];
    for (int i = 5; i >= 0; --i) {
        double s = m[i][6];
        for (int k = i + 1; k < 6; ++k) s -= m[i][k] * x[k];
        x[i] = s / (std::fabs(m[i][i]) > 1e-300 ? m[i][i] : 1e-300);
    }
    return {{x[0], x[1], x[2]}, {x[3], x[4], x[5]}};
}

// Spatial force of a force F applied at point p plus a pure torque.
inline SV forceAt(const V3d& F, const V3d& p, const V3d& torque) { return {add(cross(p, F), torque), F}; }

} // namespace sicdm::spatial

#pragma once

// Small, header-only linear algebra for the viewport.
// Conventions: right-handed, Z-up (CAD convention), column-major Mat4
// (matches OpenGL / Metal uniform layout), column vectors (M * v).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>

namespace sicdm {

inline constexpr float kPi = 3.14159265358979323846f;

constexpr float radians(float deg) noexcept { return deg * (kPi / 180.0f); }
constexpr float degrees(float rad) noexcept { return rad * (180.0f / kPi); }

template <typename T>
constexpr T lerp(T a, T b, float t) noexcept { return a + (b - a) * t; }

// ---------------------------------------------------------------------------
struct Vec2 {
    float x = 0, y = 0;
};

struct Vec3 {
    float x = 0, y = 0, z = 0;

    constexpr Vec3() noexcept = default;
    constexpr Vec3(float x_, float y_, float z_) noexcept : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const noexcept { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const noexcept { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator-() const noexcept { return {-x, -y, -z}; }
    constexpr Vec3 operator*(float s) const noexcept { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(float s) const noexcept { return {x / s, y / s, z / s}; }
    constexpr Vec3& operator+=(const Vec3& o) noexcept { x += o.x; y += o.y; z += o.z; return *this; }
    constexpr Vec3& operator-=(const Vec3& o) noexcept { x -= o.x; y -= o.y; z -= o.z; return *this; }
    constexpr Vec3& operator*=(float s) noexcept { x *= s; y *= s; z *= s; return *this; }
};

constexpr Vec3 operator*(float s, const Vec3& v) noexcept { return v * s; }

constexpr float dot(const Vec3& a, const Vec3& b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float length(const Vec3& v) noexcept { return std::sqrt(dot(v, v)); }
inline Vec3 normalize(const Vec3& v) noexcept {
    const float len = length(v);
    return len > 0.0f ? v / len : Vec3{};
}
inline Vec3 vmin(const Vec3& a, const Vec3& b) noexcept { return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)}; }
inline Vec3 vmax(const Vec3& a, const Vec3& b) noexcept { return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)}; }

struct Vec4 {
    float x = 0, y = 0, z = 0, w = 0;
};

// ---------------------------------------------------------------------------
struct Mat4 {
    // m[column][row]
    std::array<std::array<float, 4>, 4> m{};

    static constexpr Mat4 identity() noexcept {
        Mat4 r;
        r.m[0][0] = r.m[1][1] = r.m[2][2] = r.m[3][3] = 1.0f;
        return r;
    }

    const float* data() const noexcept { return &m[0][0]; }

    constexpr Mat4 operator*(const Mat4& o) const noexcept {
        Mat4 r;
        for (int c = 0; c < 4; ++c)
            for (int rr = 0; rr < 4; ++rr) {
                float s = 0.0f;
                for (int k = 0; k < 4; ++k) s += m[k][rr] * o.m[c][k];
                r.m[c][rr] = s;
            }
        return r;
    }

    constexpr Vec4 operator*(const Vec4& v) const noexcept {
        return {
            m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z + m[3][0] * v.w,
            m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z + m[3][1] * v.w,
            m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z + m[3][2] * v.w,
            m[0][3] * v.x + m[1][3] * v.y + m[2][3] * v.z + m[3][3] * v.w,
        };
    }

    constexpr Vec3 transformPoint(const Vec3& p) const noexcept {
        const Vec4 r = (*this) * Vec4{p.x, p.y, p.z, 1.0f};
        return r.w != 0.0f ? Vec3{r.x / r.w, r.y / r.w, r.z / r.w} : Vec3{r.x, r.y, r.z};
    }

    constexpr Vec3 transformDir(const Vec3& d) const noexcept {
        const Vec4 r = (*this) * Vec4{d.x, d.y, d.z, 0.0f};
        return {r.x, r.y, r.z};
    }
};

inline Mat4 translate(const Vec3& t) noexcept {
    Mat4 r = Mat4::identity();
    r.m[3][0] = t.x;
    r.m[3][1] = t.y;
    r.m[3][2] = t.z;
    return r;
}

inline Mat4 scale(const Vec3& s) noexcept {
    Mat4 r = Mat4::identity();
    r.m[0][0] = s.x;
    r.m[1][1] = s.y;
    r.m[2][2] = s.z;
    return r;
}

// View matrix looking from `eye` to `target` with `up` hint.
inline Mat4 lookAt(const Vec3& eye, const Vec3& target, const Vec3& up) noexcept {
    const Vec3 f = normalize(target - eye);
    const Vec3 s = normalize(cross(f, up));
    const Vec3 u = cross(s, f);

    Mat4 r = Mat4::identity();
    r.m[0][0] = s.x; r.m[1][0] = s.y; r.m[2][0] = s.z;
    r.m[0][1] = u.x; r.m[1][1] = u.y; r.m[2][1] = u.z;
    r.m[0][2] = -f.x; r.m[1][2] = -f.y; r.m[2][2] = -f.z;
    r.m[3][0] = -dot(s, eye);
    r.m[3][1] = -dot(u, eye);
    r.m[3][2] = dot(f, eye);
    return r;
}

// OpenGL-style clip space (z in [-1, 1]).
inline Mat4 perspective(float fovyRad, float aspect, float zNear, float zFar) noexcept {
    const float f = 1.0f / std::tan(fovyRad * 0.5f);
    Mat4 r;
    r.m[0][0] = f / aspect;
    r.m[1][1] = f;
    r.m[2][2] = (zFar + zNear) / (zNear - zFar);
    r.m[2][3] = -1.0f;
    r.m[3][2] = (2.0f * zFar * zNear) / (zNear - zFar);
    return r;
}

inline Mat4 orthographic(float left, float right, float bottom, float top, float zNear, float zFar) noexcept {
    Mat4 r = Mat4::identity();
    r.m[0][0] = 2.0f / (right - left);
    r.m[1][1] = 2.0f / (top - bottom);
    r.m[2][2] = -2.0f / (zFar - zNear);
    r.m[3][0] = -(right + left) / (right - left);
    r.m[3][1] = -(top + bottom) / (top - bottom);
    r.m[3][2] = -(zFar + zNear) / (zFar - zNear);
    return r;
}

// General 4x4 inverse (cofactor expansion). Returns identity if singular.
inline Mat4 inverse(const Mat4& a) noexcept {
    const float* s = a.data();
    float inv[16];
    inv[0] = s[5] * s[10] * s[15] - s[5] * s[11] * s[14] - s[9] * s[6] * s[15] + s[9] * s[7] * s[14] + s[13] * s[6] * s[11] - s[13] * s[7] * s[10];
    inv[4] = -s[4] * s[10] * s[15] + s[4] * s[11] * s[14] + s[8] * s[6] * s[15] - s[8] * s[7] * s[14] - s[12] * s[6] * s[11] + s[12] * s[7] * s[10];
    inv[8] = s[4] * s[9] * s[15] - s[4] * s[11] * s[13] - s[8] * s[5] * s[15] + s[8] * s[7] * s[13] + s[12] * s[5] * s[11] - s[12] * s[7] * s[9];
    inv[12] = -s[4] * s[9] * s[14] + s[4] * s[10] * s[13] + s[8] * s[5] * s[14] - s[8] * s[6] * s[13] - s[12] * s[5] * s[10] + s[12] * s[6] * s[9];
    inv[1] = -s[1] * s[10] * s[15] + s[1] * s[11] * s[14] + s[9] * s[2] * s[15] - s[9] * s[3] * s[14] - s[13] * s[2] * s[11] + s[13] * s[3] * s[10];
    inv[5] = s[0] * s[10] * s[15] - s[0] * s[11] * s[14] - s[8] * s[2] * s[15] + s[8] * s[3] * s[14] + s[12] * s[2] * s[11] - s[12] * s[3] * s[10];
    inv[9] = -s[0] * s[9] * s[15] + s[0] * s[11] * s[13] + s[8] * s[1] * s[15] - s[8] * s[3] * s[13] - s[12] * s[1] * s[11] + s[12] * s[3] * s[9];
    inv[13] = s[0] * s[9] * s[14] - s[0] * s[10] * s[13] - s[8] * s[1] * s[14] + s[8] * s[2] * s[13] + s[12] * s[1] * s[10] - s[12] * s[2] * s[9];
    inv[2] = s[1] * s[6] * s[15] - s[1] * s[7] * s[14] - s[5] * s[2] * s[15] + s[5] * s[3] * s[14] + s[13] * s[2] * s[7] - s[13] * s[3] * s[6];
    inv[6] = -s[0] * s[6] * s[15] + s[0] * s[7] * s[14] + s[4] * s[2] * s[15] - s[4] * s[3] * s[14] - s[12] * s[2] * s[7] + s[12] * s[3] * s[6];
    inv[10] = s[0] * s[5] * s[15] - s[0] * s[7] * s[13] - s[4] * s[1] * s[15] + s[4] * s[3] * s[13] + s[12] * s[1] * s[7] - s[12] * s[3] * s[5];
    inv[14] = -s[0] * s[5] * s[14] + s[0] * s[6] * s[13] + s[4] * s[1] * s[14] - s[4] * s[2] * s[13] - s[12] * s[1] * s[6] + s[12] * s[2] * s[5];
    inv[3] = -s[1] * s[6] * s[11] + s[1] * s[7] * s[10] + s[5] * s[2] * s[11] - s[5] * s[3] * s[10] - s[9] * s[2] * s[7] + s[9] * s[3] * s[6];
    inv[7] = s[0] * s[6] * s[11] - s[0] * s[7] * s[10] - s[4] * s[2] * s[11] + s[4] * s[3] * s[10] + s[8] * s[2] * s[7] - s[8] * s[3] * s[6];
    inv[11] = -s[0] * s[5] * s[11] + s[0] * s[7] * s[9] + s[4] * s[1] * s[11] - s[4] * s[3] * s[9] - s[8] * s[1] * s[7] + s[8] * s[3] * s[5];
    inv[15] = s[0] * s[5] * s[10] - s[0] * s[6] * s[9] - s[4] * s[1] * s[10] + s[4] * s[2] * s[9] + s[8] * s[1] * s[6] - s[8] * s[2] * s[5];

    const float det = s[0] * inv[0] + s[1] * inv[4] + s[2] * inv[8] + s[3] * inv[12];
    if (std::fabs(det) < 1e-12f) return Mat4::identity();

    const float invDet = 1.0f / det;
    Mat4 r;
    float* d = &r.m[0][0];
    for (int i = 0; i < 16; ++i) d[i] = inv[i] * invDet;
    return r;
}

// ---------------------------------------------------------------------------
struct AABB {
    Vec3 min{1e30f, 1e30f, 1e30f};
    Vec3 max{-1e30f, -1e30f, -1e30f};

    bool valid() const noexcept { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }
    void expand(const Vec3& p) noexcept { min = vmin(min, p); max = vmax(max, p); }
    void expand(const AABB& b) noexcept { if (b.valid()) { expand(b.min); expand(b.max); } }
    Vec3 center() const noexcept { return (min + max) * 0.5f; }
    float radius() const noexcept { return length(max - min) * 0.5f; }
};

// ---------------------------------------------------------------------------
struct Ray {
    Vec3 origin;
    Vec3 dir; // normalised

    Vec3 at(float t) const noexcept { return origin + dir * t; }
};

// Ray / plane intersection; returns false when parallel or behind the origin.
inline bool intersectPlane(const Ray& r, const Vec3& point, const Vec3& normal, float& t) noexcept {
    const float d = dot(r.dir, normal);
    if (std::fabs(d) < 1e-8f) return false;
    t = dot(point - r.origin, normal) / d;
    return t >= 0.0f;
}

// Moller-Trumbore ray / triangle test (double-sided).
inline bool intersectTriangle(const Ray& r, const Vec3& a, const Vec3& b, const Vec3& c, float& t) noexcept {
    const Vec3 e1 = b - a, e2 = c - a;
    const Vec3 p = cross(r.dir, e2);
    const float det = dot(e1, p);
    if (std::fabs(det) < 1e-12f) return false;
    const float inv = 1.0f / det;
    const Vec3 s = r.origin - a;
    const float u = dot(s, p) * inv;
    if (u < 0.0f || u > 1.0f) return false;
    const Vec3 q = cross(s, e1);
    const float v = dot(r.dir, q) * inv;
    if (v < 0.0f || u + v > 1.0f) return false;
    t = dot(e2, q) * inv;
    return t > 0.0f;
}

// Slab test; returns entry distance in `t`.
inline bool intersectAABB(const Ray& r, const AABB& b, float& t) noexcept {
    float tmin = 0.0f, tmax = 1e30f;
    const float o[3] = {r.origin.x, r.origin.y, r.origin.z};
    const float d[3] = {r.dir.x, r.dir.y, r.dir.z};
    const float lo[3] = {b.min.x, b.min.y, b.min.z};
    const float hi[3] = {b.max.x, b.max.y, b.max.z};
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(d[i]) < 1e-12f) {
            if (o[i] < lo[i] || o[i] > hi[i]) return false;
            continue;
        }
        float t0 = (lo[i] - o[i]) / d[i], t1 = (hi[i] - o[i]) / d[i];
        if (t0 > t1) std::swap(t0, t1);
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        if (tmin > tmax) return false;
    }
    t = tmin;
    return true;
}

} // namespace sicdm

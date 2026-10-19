#include "app/Splash.hpp"

#include "render/GL.hpp"
#include "render/Shader.hpp"
#include "ui/EmbeddedFonts.hpp"
#include "ui/Theme.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <stb_image.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#ifndef SICDM_VERSION
#define SICDM_VERSION "dev"
#endif

namespace sicdm {

namespace {

constexpr int kWidth = 960;  // window points
constexpr int kHeight = 540;
constexpr float kCinemaAspect = 2.39f;

const char* kFullscreenVS = R"(#version 410 core
out vec2 vUv;
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vUv = p;
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)";

const char* kBlitFS = R"(#version 410 core
in vec2 vUv;
out vec4 frag;
uniform sampler2D uTex;
void main() { frag = vec4(texture(uTex, vUv).rgb, 1.0); }
)";

// The scene: a four-wheeled crew rover crossing the lunar plain under a black,
// star-filled sky, a crescent Earth over the horizon. Everything is procedural
// and ray marched: a height field for the regolith (rolling plains, craters,
// boulders, the rover's tyre tracks, smooth highlands) and a signed distance
// field for the rover, lit by a low, hard sun with long shadows, regolith
// bounce light and the rover's own headlights, then graded like film.
const char* kSceneFS = R"(#version 410 core
in vec2 vUv;
out vec4 frag;
uniform vec2 uRes;
uniform float uTime;
uniform float uFade;

const float PI = 3.14159265;
const vec3 SUN = normalize(vec3(0.98, 0.27, -0.12));
const vec3 SUN_COL = vec3(1.0, 0.97, 0.93) * 3.6;
vec3 EARTH;                   // placed in frame, upper left (see main)
const float EARTH_R = 0.024;  // angular radius (rad), a little larger than life
const vec3 LAMP_COL = vec3(1.0, 0.8, 0.56);
const float SPEED = 0.32;     // rover m/s
const float WHEEL_R = 0.58;
const float FOCAL = 2.35;
const float SHIFT = 0.3;      // lens shift: rover right of frame, title left

float roverX;                 // rover pose, solved per pixel from the ground
float roverY;
float roverPitch;

// ---------------------------------------------------------------- noise
float hash12(vec2 p) {
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}
vec2 hash22(vec2 p) {
    vec3 p3 = fract(vec3(p.xyx) * vec3(0.1031, 0.1030, 0.0973));
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.xx + p3.yz) * p3.zy);
}
float hash13(vec3 p3) {
    p3 = fract(p3 * 0.1031);
    p3 += dot(p3, p3.zyx + 31.32);
    return fract((p3.x + p3.y) * p3.z);
}
vec3 hash33(vec3 p3) {
    p3 = fract(p3 * vec3(0.1031, 0.1030, 0.0973));
    p3 += dot(p3, p3.yxz + 33.33);
    return fract((p3.xxy + p3.yxx) * p3.zyx);
}
float noise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash12(i), hash12(i + vec2(1, 0)), u.x),
               mix(hash12(i + vec2(0, 1)), hash12(i + vec2(1, 1)), u.x), u.y);
}
const mat2 ROT = mat2(0.8, -0.6, 0.6, 0.8);
float fbm(vec2 p, int octaves) {
    float a = 0.5, s = 0.0;
    for (int i = 0; i < octaves; ++i) { s += a * noise(p); p = ROT * p * 2.03; a *= 0.5; }
    return s;
}

// ---------------------------------------------------------------- regolith
// One crater per cell (bowl and raised rim), centred so the rim stays in its cell.
float craters(vec2 p, float cellSize, float seed, float maxR) {
    vec2 cell = floor(p / cellSize);
    if (hash12(cell + seed + 9.1) > 0.6) return 0.0;
    float rad = maxR * (0.3 + 0.7 * pow(hash12(cell + seed + 3.7), 2.0));
    vec2 off = (hash22(cell + seed) - 0.5) * max(cellSize - 3.8 * rad, 0.0);
    vec2 c = (cell + 0.5) * cellSize + off;
    float d = length(p - c) / rad;
    float bowl = d < 1.0 ? (d * d - 1.0) * 0.24 * rad : 0.0;
    return bowl + 0.1 * rad * exp(-pow((d - 1.0) / 0.3, 2.0));
}

float boulders(vec2 p, out float mask) {
    const float CELL = 1.5;
    vec2 cell = floor(p / CELL);
    vec2 r = hash22(cell + 41.0);
    float rad = 0.04 + 0.42 * pow(hash12(cell + 7.13), 5.0);
    vec2 c = (cell + 0.3 + 0.4 * r) * CELL;
    vec2 d = p - c;
    float ang = atan(d.y, d.x);
    rad *= 0.85 + 0.25 * sin(ang * 3.0 + r.x * 6.0) * sin(ang * 2.0 + r.y * 5.0);  // angular, not round
    float q = 1.0 - dot(d, d) / (rad * rad);
    mask = clamp(q * 4.0, 0.0, 1.0);
    return q > 0.0 ? rad * 0.7 * pow(q, 0.35) : 0.0;
}

float ground(vec2 p, bool detail) {
    float path = smoothstep(1.8, 3.0, abs(p.y));            // the rover drives along z = 0
    float h = (fbm(p * 0.022 + 4.0, 3) - 0.5) * 2.4;
    h += (fbm(p * 0.14 + 1.7, 3) - 0.5) * mix(0.1, 0.45, path);
    float far = length(p - vec2(roverX, 0.0));
    float rim = smoothstep(60.0, 230.0, far);                 // smooth highlands far away
    if (rim > 0.0) h += rim * (pow(fbm(p * 0.007 + 9.0, 4), 1.6) * 95.0 - 12.0);
    float mid = smoothstep(18.0, 70.0, far) * (1.0 - rim);
    if (mid > 0.0) h += mid * pow(fbm(p * 0.035 + 2.0, 3), 2.0) * 9.0;
    h += (craters(p, 10.0, 0.0, 2.4) + craters(p, 2.6, 17.0, 0.55)) * path;
    float m;
    h += boulders(p, m) * path;
    // Tyre tracks with tread prints behind the rover.
    float behind = roverX - 1.42 - p.x;
    if (behind > 0.0) {
        float rut = exp(-pow((abs(p.y) - 1.22) / 0.17, 6.0));
        float print = 0.75 + 0.25 * sin(p.x * 28.0 + (p.y > 0.0 ? 1.5 : 0.0));
        h -= 0.035 * rut * print * exp(-behind * 0.02);
    }
    if (detail) {
        h += (fbm(p * 3.1, 3) - 0.5) * 0.03;
        h += (noise(p * 17.0) - 0.5) * 0.008;
    }
    return h;
}

// ---------------------------------------------------------------- rover SDF
float sdBox(vec3 p, vec3 b) {
    vec3 q = abs(p) - b;
    return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);
}
float sdRBox(vec3 p, vec3 b, float r) { return sdBox(p, b - r) - r; }
float sdRect(vec2 p, vec2 b, float r) {
    vec2 q = abs(p) - b + r;
    return length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - r;
}
float sdCapsule(vec3 p, vec3 a, vec3 b, float r) {
    vec3 pa = p - a, ba = b - a;
    float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    return length(pa - ba * h) - r;
}
float sdCyl(vec2 radial, float axial, float r, float h) {
    vec2 d = abs(vec2(length(radial), axial)) - vec2(r, h);
    return min(max(d.x, d.y), 0.0) + length(max(d, 0.0));
}
vec2 opU(vec2 a, vec2 b) { return a.x < b.x ? a : b; }

// Materials: 1 light panels, 2 charcoal, 3 tyre rubber, 4 gold rim,
// 5 glass, 6 headlamp, 7 grey metal.
vec2 wheel(vec3 p, float spin) {                              // z points outward
    float bound = length(p) - 0.66;
    if (bound > 0.1) return vec2(bound, 3.0);
    float c = cos(spin), s = sin(spin);
    p.xy = mat2(c, -s, s, c) * p.xy;
    float r = length(p.xy);
    float a = atan(p.y, p.x);
    vec2 res = vec2(sdRect(vec2(r - 0.47, p.z), vec2(0.1, 0.2), 0.08), 3.0);     // carcass
    const float N = 26.0;
    float seg = 2.0 * PI / N;
    float ta = mod(a + (p.z > 0.0 ? 0.5 * seg : 0.0) + 0.5 * seg, seg) - 0.5 * seg;
    res.x = min(res.x, sdBox(vec3(ta * r, r - 0.565, abs(p.z) - 0.1), vec3(0.042, 0.022, 0.085)) - 0.006);
    float rim = sdCyl(p.xy, p.z - 0.02, 0.385, 0.15);                              // deep gold dish
    rim = max(rim, -sdCyl(p.xy, p.z - 0.2, 0.335, 0.12));
    res = opU(res, vec2(rim, 4.0));
    float hub = sdCyl(p.xy, p.z - 0.1, 0.11, 0.04);
    float la = mod(a + PI / 5.0, 2.0 * PI / 5.0) - PI / 5.0;
    float lug = length(vec3(r * cos(la) - 0.07, r * sin(la), p.z - 0.14)) - 0.016;
    res = opU(res, vec2(min(hub, lug), 7.0));
    return res;
}

vec2 rover(vec3 p) {
    float spin = -roverX / WHEEL_R * sign(p.x);
    vec3 w = vec3(abs(p.x) - 1.42, p.y - WHEEL_R, abs(p.z) - 1.22);              // four wheels, mirrored
    vec2 res = wheel(w, spin);
    vec3 q = vec3(abs(p.x), p.y, abs(p.z));

    // Fenders and suspension.
    float fender = max(abs(length(w.xy) - 0.68) - 0.025, abs(w.z + 0.04) - 0.2);
    fender = max(fender, 0.32 - w.y);
    res = opU(res, vec2(fender, 2.0));
    float sus = sdCapsule(q, vec3(0.9, 0.82, 0.78), vec3(1.42, WHEEL_R, 0.98), 0.05);
    sus = min(sus, sdCapsule(q, vec3(0.95, 1.02, 0.8), vec3(1.42, 0.78, 0.98), 0.04));
    res = opU(res, vec2(sus, 7.0));

    // Chassis, cabin with a raked windscreen, hood, nose.
    res = opU(res, vec2(sdRBox(p - vec3(0.0, 0.98, 0.0), vec3(1.98, 0.2, 0.86), 0.05), 2.0));
    float cabin = sdRBox(p - vec3(-0.15, 1.47, 0.0), vec3(1.6, 0.32, 0.9), 0.07);
    cabin = max(cabin, dot(p - vec3(1.0, 1.79, 0.0), normalize(vec3(0.75, 1.0, 0.0))));
    cabin = max(cabin, dot(q - vec3(0.0, 1.79, 0.62), normalize(vec3(0.0, 1.0, 0.9))));
    res = opU(res, vec2(cabin, 1.0));
    float hood = sdRBox(p - vec3(1.58, 1.12, 0.0), vec3(0.45, 0.16, 0.82), 0.05);
    hood = max(hood, dot(p - vec3(2.0, 1.22, 0.0), normalize(vec3(1.0, 1.4, 0.0))));
    res = opU(res, vec2(hood, 1.0));
    res = opU(res, vec2(sdRBox(p - vec3(2.0, 0.98, 0.0), vec3(0.07, 0.16, 0.74), 0.03), 2.0));
    // Headlamp clusters, two lamps each.
    vec3 l = vec3(p.x - 2.07, p.y - 1.0, abs(p.z) - 0.5);
    res = opU(res, vec2(sdRBox(l, vec3(0.03, 0.075, 0.2), 0.02), 2.0));
    vec3 lens = vec3(l.x - 0.02, l.y, abs(l.z) - 0.095);
    res = opU(res, vec2(sdCyl(lens.yz, lens.x, 0.055, 0.018), 6.0));
    // Side windows, rear module, side rails.
    res = opU(res, vec2(sdRBox(vec3(p.x + 0.5, p.y - 1.55, q.z - 0.86), vec3(0.75, 0.1, 0.06), 0.03), 5.0));
    res = opU(res, vec2(sdRBox(p - vec3(-1.8, 1.35, 0.0), vec3(0.12, 0.3, 0.78), 0.04), 2.0));
    res = opU(res, vec2(sdRBox(vec3(p.x, p.y - 0.8, q.z - 0.84), vec3(0.75, 0.04, 0.06), 0.02), 7.0));
    // Roof: rack, sensor pod, antenna, dish, mast camera.
    res = opU(res, vec2(sdBox(p - vec3(-0.4, 1.83, 0.0), vec3(0.85, 0.025, 0.62)), 7.0));
    res = opU(res, vec2(sdRBox(p - vec3(0.5, 1.86, 0.0), vec3(0.22, 0.07, 0.4), 0.03), 2.0));
    res = opU(res, vec2(min(sdCapsule(p, vec3(-1.15, 1.84, -0.6), vec3(-1.15, 2.45, -0.6), 0.012),
                            length(p - vec3(-1.15, 2.45, -0.6)) - 0.03), 7.0));
    res = opU(res, vec2(sdCapsule(p, vec3(-0.6, 1.84, 0.45), vec3(-0.6, 1.98, 0.45), 0.025), 7.0));
    vec3 dsh = p - vec3(-0.6, 2.02, 0.45);
    float tc = cos(0.6), ts = sin(0.6);
    dsh.xy = mat2(tc, -ts, ts, tc) * dsh.xy;
    res = opU(res, vec2(sdCyl(dsh.xz, dsh.y, 0.2, 0.012), 1.0));
    res = opU(res, vec2(sdCapsule(p, vec3(1.0, 1.72, 0.66), vec3(1.0, 2.08, 0.66), 0.025), 7.0));
    res = opU(res, vec2(sdRBox(p - vec3(1.03, 2.13, 0.66), vec3(0.06, 0.05, 0.12), 0.015), 2.0));
    return res;
}

vec3 toRover(vec3 p) {
    p -= vec3(roverX, roverY, 0.0);
    float c = cos(-roverPitch), s = sin(-roverPitch);
    p.xy = mat2(c, -s, s, c) * p.xy;
    return p;
}
vec3 fromRover(vec3 p) {
    float c = cos(roverPitch), s = sin(roverPitch);
    p.xy = mat2(c, -s, s, c) * p.xy;
    return p + vec3(roverX, roverY, 0.0);
}
const vec3 BOUND_C = vec3(0.0, 1.2, 0.0);
const float BOUND_R = 2.9;

vec2 roverWorld(vec3 p) { return rover(toRover(p)); }

bool hitBound(vec3 ro, vec3 rd, out float t0, out float t1) {
    vec3 c = vec3(roverX, roverY, 0.0) + BOUND_C;
    vec3 oc = ro - c;
    float b = dot(oc, rd);
    float h = b * b - dot(oc, oc) + BOUND_R * BOUND_R;
    if (h < 0.0) return false;
    h = sqrt(h);
    t0 = max(-b - h, 0.0);
    t1 = -b + h;
    return t1 > 0.0;
}

vec2 traceRover(vec3 ro, vec3 rd) {
    float t0, t1;
    if (!hitBound(ro, rd, t0, t1)) return vec2(-1.0);
    float t = t0;
    for (int i = 0; i < 120; ++i) {
        vec2 h = roverWorld(ro + rd * t);
        if (h.x < 0.0007 * t) return vec2(t, h.y);
        t += h.x;
        if (t > t1) break;
    }
    return vec2(-1.0);
}

float traceGround(vec3 ro, vec3 rd, float tmax) {
    float t = 0.1, lastT = t;
    for (int i = 0; i < 160; ++i) {
        vec3 p = ro + rd * t;
        float h = p.y - ground(p.xz, false);
        if (h < 0.0) {                                      // refine between steps
            for (int k = 0; k < 6; ++k) {
                float m = 0.5 * (lastT + t);
                vec3 q = ro + rd * m;
                if (q.y - ground(q.xz, false) < 0.0) t = m; else lastT = m;
            }
            return t;
        }
        lastT = t;
        t += max(0.4 * h, 0.006 * t + 0.01);
        if (t > tmax) break;
    }
    return -1.0;
}

vec3 groundNormal(vec2 p, float t) {
    float e = 0.004 + t * 0.0015;
    float hx = ground(p + vec2(e, 0), true) - ground(p - vec2(e, 0), true);
    float hz = ground(p + vec2(0, e), true) - ground(p - vec2(0, e), true);
    return normalize(vec3(-hx, 2.0 * e, -hz));
}

vec3 roverNormal(vec3 p) {
    const vec2 k = vec2(1, -1) * 0.0007;
    return normalize(k.xyy * roverWorld(p + k.xyy).x + k.yyx * roverWorld(p + k.yyx).x +
                     k.yxy * roverWorld(p + k.yxy).x + k.xxx * roverWorld(p + k.xxx).x);
}

// No air: shadows are nearly hard.
float shadow(vec3 ro, vec3 rd) {
    float res = 1.0;
    float t0, t1;
    if (hitBound(ro, rd, t0, t1)) {
        float t = max(t0, 0.01);
        for (int i = 0; i < 56 && t < t1; ++i) {
            float h = roverWorld(ro + rd * t).x;
            res = min(res, 32.0 * h / t);
            if (res < 0.002) return 0.0;
            t += clamp(h, 0.008, 0.25);
        }
    }
    float t = 0.04;
    for (int i = 0; i < 36; ++i) {
        vec3 p = ro + rd * t;
        float h = p.y - ground(p.xz, false);
        res = min(res, 22.0 * h / t);
        if (res < 0.002) return 0.0;
        t += clamp(h, 0.05, 6.0);
        if (t > 150.0) break;
    }
    return clamp(res, 0.0, 1.0);
}

float roverAO(vec3 p, vec3 n) {
    float occ = 0.0, w = 1.0;
    for (int i = 1; i <= 5; ++i) {
        float h = 0.035 * float(i);
        occ += w * (h - roverWorld(p + n * h).x);
        w *= 0.7;
    }
    return clamp(1.0 - 3.0 * occ, 0.0, 1.0);
}

// ---------------------------------------------------------------- sky
vec3 space(vec3 rd) {
    vec3 col = vec3(0.0012, 0.0015, 0.0022);
    // Faint Milky Way band.
    vec3 bn = normalize(vec3(0.35, 0.75, -0.55));
    float band = exp(-pow(dot(rd, bn) / 0.2, 2.0));
    vec2 uvs = vec2(atan(rd.z, rd.x), asin(clamp(rd.y, -1.0, 1.0)));
    col += vec3(0.55, 0.58, 0.72) * band * pow(fbm(uvs * vec2(7.0, 11.0) + 3.0, 4), 2.5) * 0.06;
    // Stars in two layers.
    for (int layer = 0; layer < 2; ++layer) {
        vec3 q = rd * (layer == 0 ? 160.0 : 380.0);
        vec3 cell = floor(q);
        vec3 f = fract(q) - 0.5;
        float h = hash13(cell + float(layer) * 19.0);
        float thr = layer == 0 ? 0.98 : 0.96;
        if (h > thr) {
            vec3 o = (hash33(cell) - 0.5) * 0.6;
            float d = length(f - o);
            float b = pow((h - thr) / (1.0 - thr), 3.0) * (layer == 0 ? 4.5 : 1.8);
            vec3 tint = mix(vec3(1.0, 0.86, 0.72), vec3(0.72, 0.82, 1.0), hash13(cell + 1.7));
            col += tint * b * exp(-d * d * 45.0);
        }
    }
    // Crescent Earth with its blue limb.
    float ce = dot(rd, EARTH);
    if (ce > cos(EARTH_R * 1.6)) {
        vec3 ex = normalize(cross(EARTH, vec3(0, 1, 0)));
        vec3 ey = cross(ex, EARTH);
        vec2 e = vec2(dot(rd, ex), dot(rd, ey)) / sin(EARTH_R);
        float rr = dot(e, e);
        vec3 n = normalize(e.x * ex + e.y * ey - EARTH * sqrt(max(1.0 - rr, 0.0)));
        float lit = dot(n, normalize(SUN + EARTH * 1.15));     // gibbous phase
        if (rr < 1.0) {
            vec2 ll = vec2(atan(n.x, n.z) + uTime * 0.01, asin(n.y));
            float land = smoothstep(0.53, 0.58, fbm(ll * 2.6 + 1.0, 5));
            float cloud = smoothstep(0.48, 0.75, fbm(ll * vec2(3.0, 5.0) + 6.0, 5));
            vec3 surf = mix(vec3(0.03, 0.09, 0.24), vec3(0.22, 0.24, 0.13), land);
            surf = mix(surf, vec3(0.95), cloud * 0.85);
            vec3 ec = surf * smoothstep(-0.02, 0.3, lit) * 1.5;
            ec += vec3(0.25, 0.45, 1.0) * pow(1.0 - sqrt(max(1.0 - rr, 0.0)), 3.0) * smoothstep(-0.1, 0.4, lit) * 1.5;
            col = mix(col, ec + vec3(0.002, 0.003, 0.006), smoothstep(1.0, 0.94, rr));  // night side: faint
        }
        float rim = sqrt(rr) - 1.0;
        if (rim > 0.0) col += vec3(0.2, 0.4, 1.0) * exp(-rim * 18.0) * smoothstep(-0.2, 0.5, lit) * 0.25;
    }
    return col;
}

// ---------------------------------------------------------------- shading
vec3 material(float id, vec3 lp, out float spec, out float rough, out float metal, out vec3 emit) {
    spec = 0.04; rough = 0.6; metal = 0.0; emit = vec3(0.0);
    if (id < 1.5) {                                           // panels with seams; windscreen glass
        if (dot(lp - vec3(1.0, 1.79, 0.0), normalize(vec3(0.75, 1.0, 0.0))) > -0.012 && abs(lp.z) < 0.72 &&
            lp.y > 1.5) {
            spec = 0.9; rough = 0.04;
            return vec3(0.012, 0.014, 0.018);
        }
        rough = 0.42; spec = 0.06;
        vec2 seam = abs(fract(vec2(lp.x * 1.6, lp.y * 3.0) + 0.5) - 0.5);
        float line = smoothstep(0.0, 0.01, min(seam.x, seam.y));
        return vec3(0.47, 0.48, 0.49) * mix(0.6, 1.0, line);
    }
    if (id < 2.5) { spec = 0.06; rough = 0.45; return vec3(0.045, 0.047, 0.052); }
    if (id < 3.5) { spec = 0.035; rough = 0.8; return vec3(0.02, 0.02, 0.022); }
    if (id < 4.5) { metal = 1.0; spec = 1.0; rough = 0.42; return vec3(0.78, 0.57, 0.3); }
    if (id < 5.5) { spec = 0.9; rough = 0.04; return vec3(0.01, 0.012, 0.016); }
    if (id < 6.5) { emit = LAMP_COL * 9.0; return vec3(0.9); }
    metal = 0.8; spec = 0.6; rough = 0.35;
    return vec3(0.34, 0.34, 0.36);
}

vec3 lampPos(float side) { return fromRover(vec3(2.1, 1.0, 0.5 * side)); }
vec3 lampDir() { return normalize(vec3(cos(roverPitch), sin(roverPitch) - 0.12, 0.0)); }

// Headlight beams on the ground ahead of the rover.
vec3 headlights(vec3 p, vec3 n) {
    vec3 sum = vec3(0.0);
    for (int i = 0; i < 2; ++i) {
        vec3 L = lampPos(i == 0 ? 1.0 : -1.0);
        vec3 v = p - L;
        float d2 = dot(v, v);
        vec3 dir = v * inversesqrt(d2);
        float cone = smoothstep(0.84, 0.96, dot(dir, lampDir()));
        sum += LAMP_COL * cone * max(dot(n, -dir), 0.0) * 14.0 / (1.0 + d2);
    }
    return sum;
}

vec3 light(vec3 n, vec3 rd, vec3 alb, float spec, float rough, float metal, float ao, float sha) {
    float ndl = max(dot(n, SUN), 0.0);
    float dif = ndl * sha;
    vec3 hv = normalize(SUN - rd);
    float shin = mix(600.0, 6.0, rough);
    float sp = pow(max(dot(n, hv), 0.0), shin) * (shin + 8.0) / 40.0 * dif;
    float fre = pow(1.0 - max(dot(n, -rd), 0.0), 5.0);
    vec3 specCol = mix(vec3(spec), alb, metal);
    vec3 col = alb * (1.0 - metal) * SUN_COL * dif;
    col += SUN_COL * sp * (specCol + (1.0 - specCol) * fre) * 0.5;
    // Bounce from the sunlit regolith reaches faces turned away from the sky.
    float bounce = clamp(0.55 - 0.45 * n.y, 0.0, 1.0);
    vec3 amb = vec3(0.3, 0.3, 0.31) * 0.16 * bounce + vec3(0.012, 0.014, 0.02);
    col += alb * mix(vec3(1.0), alb * 1.5, metal) * amb * ao;
    vec3 r = reflect(rd, n);
    vec3 env = r.y < 0.0 ? vec3(0.09, 0.09, 0.095) : space(r) * 0.5;
    col += env * (specCol + (1.0 - specCol) * fre) * (1.0 - rough) * ao;
    return col;
}

vec3 aces(vec3 x) {
    return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
}

void main() {
    vec2 frag2 = vUv * uRes;
    float time = uTime;

    // Rover pose from the ground under its wheels.
    roverX = 4.0 + SPEED * time;
    float hf = ground(vec2(roverX + 1.42, 0.0), false);
    float hr = ground(vec2(roverX - 1.42, 0.0), false);
    roverPitch = atan(hf - hr, 2.84);
    roverY = 0.5 * (hf + hr) - 0.015;

    // Ground-level camera, slowly pushing in and drifting around the rover.
    float ang = radians(30.0 + 6.0 * sin(time * 0.1 + 0.3));
    vec3 target = vec3(roverX + 0.1, roverY + 1.0, 0.0);
    float dist = 10.8 - 1.0 * time / (time + 8.0);
    vec3 ro = target + vec3(cos(ang) * dist, 0.0, sin(ang) * dist);
    ro.y = max(ground(ro.xz, false) + 0.5, roverY + 0.6);
    vec3 fw = normalize(target - ro);
    vec3 rt = normalize(cross(fw, vec3(0, 1, 0)));
    vec3 up = cross(rt, fw);
    vec2 sp = (frag2 - 0.5 * uRes) / uRes.y;
    vec2 lensUv = sp - vec2(SHIFT, 0.0);
    vec3 rd = normalize(fw * FOCAL + lensUv.x * rt + lensUv.y * up);
    // Earth hangs over the horizon at the upper left of the frame.
    vec3 fw0 = normalize(vec3(-0.866, 0.0, -0.5));
    EARTH = normalize(fw0 * 0.95 + normalize(cross(vec3(0, 1, 0), fw0)) * 0.36 + vec3(0.0, 0.13, 0.0));

    vec3 col;
    vec2 rv = traceRover(ro, rd);
    float tmax = rv.x > 0.0 ? rv.x : 700.0;
    float tg = traceGround(ro, rd, tmax);
    float hitT = 1e5;
    if (tg > 0.0) {
        hitT = tg;
        vec3 p = ro + rd * tg;
        vec3 n = groundNormal(p.xz, tg);
        float bm;
        boulders(p.xz, bm);
        bm *= smoothstep(1.8, 3.0, abs(p.z));
        float v = fbm(p.xz * 0.35, 4);
        vec3 alb = vec3(0.25, 0.25, 0.26) * (0.6 + 0.7 * v);
        alb = mix(alb, vec3(0.22, 0.22, 0.23) * (0.8 + 0.5 * noise(p.xz * 11.0)), bm);
        float sha = shadow(p + n * 0.02, SUN);
        float ao = 1.0;
        float cd = length(p - (vec3(roverX, roverY, 0.0) + BOUND_C));
        if (cd < BOUND_R + 1.0) ao = 0.3 + 0.7 * clamp(roverWorld(p + vec3(0, 0.3, 0)).x / 0.3, 0.0, 1.0);
        // Regolith scatters like the Moon: flat (Lommel-Seeliger) with an opposition glow.
        float mu0 = max(dot(n, SUN), 0.0), mu = max(dot(n, -rd), 0.05);
        float ls = 2.0 * mu0 / (mu0 + mu);
        float lunar = mix(mu0, ls * 0.5, 0.55) * (1.0 + 0.25 * pow(max(dot(-rd, SUN), 0.0), 6.0));
        col = alb * SUN_COL * lunar * sha;
        col += alb * vec3(0.3, 0.3, 0.31) * 0.05 * ao + alb * vec3(0.01, 0.012, 0.018);
        col += alb * headlights(p, n) * ao;
        col *= ao * 0.4 + 0.6;
    } else if (rv.x > 0.0) {
        hitT = rv.x;
        vec3 p = ro + rd * rv.x;
        vec3 n = roverNormal(p);
        vec3 lp = toRover(p);
        float spec, rough, metal;
        vec3 emit;
        vec3 alb = material(rv.y, lp, spec, rough, metal, emit);
        alb = mix(alb, vec3(0.28, 0.28, 0.29), smoothstep(1.0, 0.25, lp.y) * 0.35 * step(rv.y, 2.5)); // dust
        float sha = shadow(p + n * 0.003, SUN);
        float ao = roverAO(p, n);
        col = light(n, rd, alb, spec, rough, metal, ao, sha) + emit;
    } else {
        col = space(rd);
    }
    if (hitT < 1e4) col = mix(col, vec3(0.012, 0.012, 0.014), 1.0 - exp(-hitT * 0.0011)); // a breath of depth

    // Headlamp glow and an anamorphic streak, where the lamps face the lens.
    for (int i = 0; i < 2; ++i) {
        vec3 L = lampPos(i == 0 ? 1.0 : -1.0);
        vec3 v = L - ro;
        float z = dot(v, fw);
        if (z < 0.1) continue;
        vec2 s = vec2(dot(v, rt), dot(v, up)) / z * FOCAL + vec2(SHIFT, 0.0);
        vec2 d = sp - s;
        float facing = smoothstep(0.0, 0.4, dot(lampDir(), normalize(ro - L)));
        float r2 = dot(d, d);
        vec3 glow = LAMP_COL * (exp(-r2 * 9000.0) * 3.0 + 0.0006 / (r2 + 0.0006) * 0.18);
        glow += vec3(0.55, 0.7, 1.0) * exp(-abs(d.y) * 900.0) * exp(-abs(d.x) * 6.0) * 0.22;
        col += glow * facing;
    }

    // Grade: exposure, filmic curve, cool shadows, vignette, grain.
    col *= 1.05;
    col = aces(col);
    col = pow(col, vec3(1.0 / 2.2));
    float luma = dot(col, vec3(0.299, 0.587, 0.114));
    col += vec3(-0.004, 0.002, 0.012) * (1.0 - luma);
    col = smoothstep(0.0, 1.0, col) * 0.3 + col * 0.7;
    vec2 uv = vUv - 0.5;
    col *= 1.0 - 0.6 * dot(uv * vec2(0.85, 1.4), uv * vec2(0.85, 1.4));
    col += (hash12(frag2 + fract(time) * 437.0) - 0.5) * 0.04;
    frag = vec4(clamp(col, 0.0, 1.0) * uFade, 1.0);
}
)";

float easeOutCubic(float x) {
    x = std::clamp(x, 0.0f, 1.0f);
    return 1.0f - (1.0f - x) * (1.0f - x) * (1.0f - x);
}

ImU32 withAlpha(ImU32 c, float a) {
    const auto alpha = static_cast<ImU32>(std::clamp(a, 0.0f, 1.0f) * static_cast<float>((c >> 24) & 0xFF));
    return (c & 0x00FFFFFFu) | (alpha << 24);
}

// Makes `window` / `context` current for a scope and restores the caller's.
struct ScopedContexts {
    GLFWwindow* prevWindow;
    ImGuiContext* prevContext;
    ScopedContexts(GLFWwindow* window, ImGuiContext* context)
        : prevWindow(glfwGetCurrentContext()), prevContext(ImGui::GetCurrentContext()) {
        glfwMakeContextCurrent(window);
        ImGui::SetCurrentContext(context);
    }
    ~ScopedContexts() {
        glfwMakeContextCurrent(prevWindow);
        ImGui::SetCurrentContext(prevContext);
    }
    ScopedContexts(const ScopedContexts&) = delete;
    ScopedContexts& operator=(const ScopedContexts&) = delete;
};

} // namespace

void applyWindowIcon(GLFWwindow* window) {
#if defined(__APPLE__)
    (void)window; // the Dock shows the bundle icon (resources/macos/SICDM.icns)
#else
    GLFWimage images[3];
    unsigned char* pixels[3] = {};
    int count = 0;
    for (const fonts::Blob* blob : {&icons::sicdm_256, &icons::sicdm_64, &icons::sicdm_32}) {
        int w = 0, h = 0, n = 0;
        unsigned char* px = stbi_load_from_memory(blob->data, static_cast<int>(blob->size), &w, &h, &n, 4);
        if (!px) continue;
        images[count] = GLFWimage{w, h, px};
        pixels[count++] = px;
    }
    if (count > 0) glfwSetWindowIcon(window, count, images);
    for (int i = 0; i < count; ++i) stbi_image_free(pixels[i]);
#endif
}

Splash::Splash(float minSeconds) : m_minSeconds(minSeconds) {
    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_FLOATING, GLFW_TRUE);
    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);
#if defined(__APPLE__)
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_TRUE);
#endif
    m_window = glfwCreateWindow(kWidth, kHeight, "SI CDM", nullptr, nullptr);
    glfwDefaultWindowHints();
    if (!m_window) return;
    applyWindowIcon(m_window);

    // Centre on the primary monitor's work area.
    if (GLFWmonitor* monitor = glfwGetPrimaryMonitor()) {
        int mx = 0, my = 0, mw = 0, mh = 0, ww = 0, wh = 0;
        glfwGetMonitorWorkarea(monitor, &mx, &my, &mw, &mh);
        glfwGetWindowSize(m_window, &ww, &wh);
        if (mw > 0 && mh > 0) glfwSetWindowPos(m_window, mx + (mw - ww) / 2, my + (mh - wh) / 2);
    }

    GLFWwindow* prevWindow = glfwGetCurrentContext();
    ImGuiContext* prevContext = ImGui::GetCurrentContext();
    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);

    m_scene = std::make_unique<Shader>(kFullscreenVS, kSceneFS);
    m_blit = std::make_unique<Shader>(kFullscreenVS, kBlitFS);
    glGenVertexArrays(1, &m_vao);

    // Logo texture.
    {
        int w = 0, h = 0, n = 0;
        unsigned char* px = stbi_load_from_memory(icons::sicdm_256.data, static_cast<int>(icons::sicdm_256.size),
                                                  &w, &h, &n, 4);
        if (px) {
            glGenTextures(1, &m_logoTex);
            glBindTexture(GL_TEXTURE_2D, m_logoTex);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, px);
            glGenerateMipmap(GL_TEXTURE_2D);
            stbi_image_free(px);
        }
    }

    // Own ImGui context with the workbench fonts (the main UI's fonts are untouched).
    m_imgui = ImGui::CreateContext();
    ImGui::SetCurrentContext(m_imgui);
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    int ww = 0, fw = 0, wh = 0, fh = 0;
    glfwGetWindowSize(m_window, &ww, &wh);
    glfwGetFramebufferSize(m_window, &fw, &fh);
    const float fbScale = ww > 0 ? std::max(1.0f, static_cast<float>(fw) / static_cast<float>(ww)) : 1.0f;
    float cx = 1.0f, cy = 1.0f;
    glfwGetWindowContentScale(m_window, &cx, &cy);
    const float ui = std::max(1.0f, cx / fbScale); // GLFW_SCALE_TO_MONITOR enlarged the window by this
    auto add = [&](const fonts::Blob& blob, float size) {
        ImFontConfig cfg;
        cfg.FontDataOwnedByAtlas = false;
        cfg.OversampleH = 2;
        return io.Fonts->AddFontFromMemoryTTF(const_cast<unsigned char*>(blob.data), static_cast<int>(blob.size),
                                              size * ui * fbScale, &cfg);
    };
    m_fontBody = add(fonts::Inter_Regular, 13.0f);
    m_fontTitle = add(fonts::Inter_SemiBold, 40.0f);
    m_fontSmall = add(fonts::Inter_Regular, 11.5f);
    m_fontCaption = add(fonts::Inter_SemiBold, 10.5f);
    m_fontMono = add(fonts::JetBrainsMono_Regular, 11.5f);
    io.FontGlobalScale = 1.0f / fbScale;
    ImGui_ImplGlfw_InitForOpenGL(m_window, false);
    ImGui_ImplOpenGL3_Init("#version 410 core");

    glfwMakeContextCurrent(prevWindow);
    ImGui::SetCurrentContext(prevContext);

    m_start = m_last = glfwGetTime();
    glfwShowWindow(m_window);
    frame();
}

Splash::~Splash() {
    if (!m_window) return;
    {
        ScopedContexts scope(m_window, m_imgui);
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext(m_imgui);
        m_scene.reset();
        m_blit.reset();
        if (m_vao) glDeleteVertexArrays(1, &m_vao);
        if (m_fbo) glDeleteFramebuffers(1, &m_fbo);
        if (m_fboTex) glDeleteTextures(1, &m_fboTex);
        if (m_logoTex) glDeleteTextures(1, &m_logoTex);
        // The scope restores the caller's ImGui context; never leave the destroyed one current.
        if (scope.prevContext == m_imgui) scope.prevContext = nullptr;
    }
    glfwDestroyWindow(m_window);
}

void Splash::step(float progress, const char* status) {
    if (!m_window) return;
    m_target = std::max(m_target, std::clamp(progress, 0.0f, 1.0f));
    m_steps.push_back({m_target, status});
    frame();
}

bool Splash::skipRequested() const {
    return glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS ||
           glfwGetKey(m_window, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwGetKey(m_window, GLFW_KEY_ENTER) == GLFW_PRESS ||
           glfwGetKey(m_window, GLFW_KEY_SPACE) == GLFW_PRESS;
}

void Splash::finish() {
    if (!m_window) return;
    m_target = 1.0f;
    constexpr float kHold = 0.35f, kFade = 0.55f;
    double readyAt = -1.0;
    while (true) {
        const double now = glfwGetTime();
        const bool skip = skipRequested() || glfwWindowShouldClose(m_window);
        if (m_closing < 0.0f) {
            if (m_shown >= 0.999f && readyAt < 0.0) readyAt = now;
            const bool waited = now - m_start >= m_minSeconds && readyAt >= 0.0 && now - readyAt >= kHold;
            if (waited || skip) m_closing = 0.0f;
        }
        if (m_closing >= kFade) break;
        frame();
    }
}

void Splash::frame() {
    ScopedContexts scope(m_window, m_imgui);
    const double now = glfwGetTime();
    const float dt = static_cast<float>(std::min(now - m_last, 0.1));
    m_last = now;
    const float elapsed = static_cast<float>(now - m_start);

    // The bar follows the reported progress, paced so it fills over the minimum time.
    const float paced = std::min(m_target, elapsed / std::max(m_minSeconds - 0.4f, 0.5f));
    m_shown += (paced - m_shown) * std::min(1.0f, dt * 6.0f);
    if (m_target >= 1.0f && paced >= 1.0f && 1.0f - m_shown < 0.002f) m_shown = 1.0f;
    if (m_closing >= 0.0f) {
        m_closing += dt;
        m_shown += (1.0f - m_shown) * std::min(1.0f, dt * 12.0f);
    }

    const float fadeIn = easeOutCubic(elapsed / 1.1f);
    const float fadeOut = m_closing >= 0.0f ? 1.0f - easeOutCubic(m_closing / 0.55f) : 1.0f;
    const float fade = fadeIn * fadeOut;

    int fbw = 0, fbh = 0, ww = 0, wh = 0;
    glfwGetFramebufferSize(m_window, &fbw, &fbh);
    glfwGetWindowSize(m_window, &ww, &wh);
    const float imageHeight = std::floor(static_cast<float>(fbw) / kCinemaAspect);
    const float imageTop = std::floor((static_cast<float>(fbh) - imageHeight) * 0.5f);

    glViewport(0, 0, fbw, fbh);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glClearColor(0.012f * fade, 0.013f * fade, 0.016f * fade, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    if (fade > 0.0f) drawScene(fbw, fbh, imageTop, imageHeight, fade);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    drawOverlay(fade);
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(m_window);
    glfwSetWindowOpacity(m_window, m_closing >= 0.0f ? std::max(fadeOut, 0.0f) : 1.0f);

    // Events go to the caller's contexts (the main window's callbacks use them).
    glfwMakeContextCurrent(scope.prevWindow);
    ImGui::SetCurrentContext(scope.prevContext);
    glfwPollEvents();
}

void Splash::drawScene(int fbw, int fbh, float imageTop, float imageHeight, float fade) {
    (void)fbh;
    if (!m_scene || !m_scene->valid()) return;
    const int w = std::max(16, static_cast<int>(static_cast<float>(fbw) * m_renderScale));
    const int h = std::max(9, static_cast<int>(imageHeight * m_renderScale));
    if (w != m_fboW || h != m_fboH) {
        if (!m_fbo) glGenFramebuffers(1, &m_fbo);
        if (!m_fboTex) glGenTextures(1, &m_fboTex);
        glBindTexture(GL_TEXTURE_2D, m_fboTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_fboTex, 0);
        m_fboW = w;
        m_fboH = h;
    }

    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glViewport(0, 0, m_fboW, m_fboH);
    glBindVertexArray(m_vao);
    m_scene->bind();
    m_scene->set("uRes", Vec2{static_cast<float>(m_fboW), static_cast<float>(m_fboH)});
    m_scene->set("uTime", static_cast<float>(glfwGetTime() - m_start));
    m_scene->set("uFade", fade);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glFinish();
    const float ms = std::chrono::duration<float, std::milli>(clock::now() - t0).count();
    // Keep the frame rate smooth on slower GPUs (and sharpen on fast ones).
    if (ms > 22.0f) m_renderScale = std::max(0.25f, m_renderScale * 0.85f);
    else if (ms < 8.0f) m_renderScale = std::min(1.0f, m_renderScale * 1.05f);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, static_cast<GLint>(imageTop), fbw, static_cast<GLsizei>(imageHeight));
    m_blit->bind();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_fboTex);
    glUniform1i(glGetUniformLocation(m_blit->program(), "uTex"), 0);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
    glViewport(0, 0, fbw, fbh);
}

void Splash::drawOverlay(float fade) {
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const ImVec2 size = ImGui::GetIO().DisplaySize;
    const float imageH = std::floor(size.x / kCinemaAspect);
    const float barTop = std::floor((size.y - imageH) * 0.5f);   // top letterbox height
    const float bottomY = barTop + imageH;                         // bottom letterbox starts here
    const float elapsed = static_cast<float>(glfwGetTime() - m_start);
    const float pad = 28.0f;

    auto text = [&](ImFont* font, float px, ImVec2 pos, ImU32 col, const char* s, float tracking = 0.0f) {
        if (tracking == 0.0f) {
            dl->AddText(font, px, pos, col, s);
            return;
        }
        for (const char* c = s; *c;) { // one UTF-8 character at a time
            const auto lead = static_cast<unsigned char>(*c);
            const int n = lead < 0x80 ? 1 : lead < 0xE0 ? 2 : lead < 0xF0 ? 3 : 4;
            dl->AddText(font, px, pos, col, c, c + n);
            pos.x += font->CalcTextSizeA(px, 1e9f, 0.0f, c, c + n).x + tracking;
            c += n;
        }
    };
    auto width = [&](ImFont* font, float px, const char* s, float tracking = 0.0f) {
        float w = font->CalcTextSizeA(px, 1e9f, 0.0f, s).x;
        if (tracking != 0.0f) w += tracking * static_cast<float>(std::char_traits<char>::length(s));
        return w;
    };
    const float scale = ImGui::GetIO().FontGlobalScale;
    const float body = m_fontBody->FontSize * scale, small = m_fontSmall->FontSize * scale;
    const float caption = m_fontCaption->FontSize * scale, mono = m_fontMono->FontSize * scale;
    const float title = m_fontTitle->FontSize * scale;
    const float ui = body / 13.0f;

    // Top letterbox: mission slate.
    {
        const float y = std::floor(barTop * 0.5f - caption * 0.5f);
        text(m_fontCaption, caption, ImVec2(pad, y), withAlpha(theme::kTextFaint, fade), "SI CDM  \xC2\xB7  ENGINEERING WORKBENCH",
             1.6f * ui);
        const char* slate = "LUNAR DAY 214  \xC2\xB7  MARE IMBRIUM  \xC2\xB7  32.8\xC2\xB0N 15.6\xC2\xB0W";
        text(m_fontMono, mono, ImVec2(size.x - pad - width(m_fontMono, mono, slate), y - 1.0f),
             withAlpha(theme::kTextFaint, fade), slate);
    }

    // Title card over the image, rising in after the scene fades up.
    {
        const float a = easeOutCubic((elapsed - 0.5f) / 1.2f) * fade;
        const float rise = (1.0f - easeOutCubic((elapsed - 0.5f) / 1.4f)) * 14.0f * ui;
        const float logo = 56.0f * ui;
        const float x = pad + 8.0f * ui;
        const float y = bottomY - 30.0f * ui - logo + rise;
        // Soft scrim so the type reads on any frame.
        const ImU32 dark = IM_COL32(4, 4, 6, static_cast<int>(150 * a));
        const ImU32 clear = IM_COL32(4, 4, 6, 0);
        dl->AddRectFilledMultiColor(ImVec2(0, bottomY - 150.0f * ui), ImVec2(size.x * 0.62f, bottomY), clear, clear, clear,
                                    dark);
        if (m_logoTex)
            dl->AddImage(static_cast<ImTextureID>(m_logoTex), ImVec2(x, y), ImVec2(x + logo, y + logo), ImVec2(0, 0),
                         ImVec2(1, 1), IM_COL32(255, 255, 255, static_cast<int>(255 * a)));
        const float tx = x + logo + 16.0f * ui;
        text(m_fontTitle, title, ImVec2(tx + 1.0f, y - 6.0f * ui + 2.0f), IM_COL32(0, 0, 0, static_cast<int>(110 * a)), "SI CDM",
             3.0f * ui);
        text(m_fontTitle, title, ImVec2(tx, y - 6.0f * ui), withAlpha(theme::rgb(244, 245, 247), a), "SI CDM", 3.0f * ui);
        text(m_fontCaption, caption * 1.15f, ImVec2(tx + 2.0f, y + title - 2.0f * ui),
             withAlpha(theme::rgb(190, 198, 210), a * 0.95f), "CAD  \xC2\xB7  CAM  \xC2\xB7  MOTION SIMULATION", 2.4f * ui);
    }

    // Bottom letterbox: progress line on the image edge, status and percentage.
    {
        const float a = fade;
        const float lineY = bottomY;
        const float fillX = size.x * m_shown;
        dl->AddRectFilled(ImVec2(0, lineY), ImVec2(size.x, lineY + 2.0f * ui), IM_COL32(255, 255, 255, static_cast<int>(18 * a)));
        dl->AddRectFilled(ImVec2(0, lineY), ImVec2(fillX, lineY + 2.0f * ui), withAlpha(theme::kAccentHover, a));
        // Glow at the leading edge.
        for (int i = 0; i < 6; ++i) {
            const float r = (3.0f + 3.0f * static_cast<float>(i)) * ui;
            dl->AddCircleFilled(ImVec2(fillX, lineY + 1.0f * ui), r,
                                withAlpha(theme::kAccentText, a * 0.10f * static_cast<float>(6 - i) / 6.0f), 24);
        }

        const char* status = m_steps.empty() ? "Starting" : m_steps.front().status.c_str();
        for (const Step& s : m_steps)
            if (s.at <= m_shown + 0.02f) status = s.status.c_str();
        if (m_closing >= 0.0f || m_shown >= 0.999f) status = "Ready";
        const float y = lineY + ((size.y - lineY) - body) * 0.5f;
        // Activity pulse before the status.
        const float pulse = 0.5f + 0.5f * std::sin(elapsed * 5.0f);
        dl->AddCircleFilled(ImVec2(pad + 3.0f * ui, y + body * 0.55f), 3.0f * ui,
                            withAlpha(m_shown >= 0.999f ? theme::kSuccess : theme::kAccentText, a * (0.45f + 0.55f * pulse)),
                            16);
        text(m_fontBody, body, ImVec2(pad + 14.0f * ui, y), withAlpha(theme::kText, a), status);
        char pct[16];
        std::snprintf(pct, sizeof pct, "%3d%%", static_cast<int>(std::round(m_shown * 100.0f)));
        const float pw = width(m_fontMono, mono, pct);
        text(m_fontMono, mono, ImVec2(size.x - pad - pw, y + 1.0f), withAlpha(theme::kText, a), pct);
        char version[48];
        std::snprintf(version, sizeof version, "Version %s", SICDM_VERSION);
        const float vw = width(m_fontSmall, small, version);
        text(m_fontSmall, small, ImVec2(size.x - pad - pw - 18.0f * ui - vw, y + 1.5f), withAlpha(theme::kTextFaint, a), version);
    }
}

} // namespace sicdm

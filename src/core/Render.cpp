#include "core/Render.hpp"

#include "core/Bvh.hpp"
#include "core/FileIO.hpp"
#include "core/Scene.hpp"
#include "core/Translate.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <map>
#include <memory>
#include <thread>

namespace sicdm {

const std::vector<std::string>& renderEnvironments() {
    static const std::vector<std::string> k = {"studio", "sky", "white", "dark"};
    return k;
}

RenderCamera renderCamera(const OrbitCamera& c) {
    RenderCamera r;
    r.position = c.position(), r.forward = c.forward(), r.right = c.right(), r.up = c.up();
    r.fovY = c.fovY();
    r.ortho = c.projection() == Projection::Orthographic;
    r.orthoHeight = 2.0f * c.distance() * std::tan(0.5f * c.fovY());
    return r;
}

RenderCamera defaultRenderCamera(const Scene& scene, float aspect) {
    OrbitCamera cam;
    cam.setAspect(aspect);
    cam.setStandardView(StandardView::Isometric, false);
    cam.fitToBounds(scene.bounds(), false);
    return renderCamera(cam);
}

namespace {

constexpr float kPi = 3.14159265358979f;

float lum(const Vec3& c) { return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z; }
Vec3 mul(const Vec3& a, const Vec3& b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
Vec3 toLinear(const Vec3& c) { return {std::pow(std::max(c.x, 0.0f), 2.2f), std::pow(std::max(c.y, 0.0f), 2.2f), std::pow(std::max(c.z, 0.0f), 2.2f)}; }
float maxc(const Vec3& c) { return std::max({c.x, c.y, c.z}); }

// PCG32: small, fast and good enough for sampling.
struct Rng {
    std::uint64_t state;
    explicit Rng(std::uint64_t seed) : state(seed * 6364136223846793005ull + 1442695040888963407ull) { next(); }
    std::uint32_t next() {
        const std::uint64_t old = state;
        state = old * 6364136223846793005ull + 1442695040888963407ull;
        const auto x = static_cast<std::uint32_t>(((old >> 18u) ^ old) >> 27u);
        const auto r = static_cast<std::uint32_t>(old >> 59u);
        return (x >> r) | (x << ((32u - r) & 31u));
    }
    float uni() { return static_cast<float>(next() >> 8) * (1.0f / 16777216.0f); }
};

// An orthonormal frame around n.
void frameOf(const Vec3& n, Vec3& t, Vec3& b) {
    const Vec3 a = std::fabs(n.x) > 0.9f ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
    t = normalize(cross(a, n));
    b = cross(n, t);
}

struct Texture {
    int w = 0, h = 0;
    std::vector<unsigned char> px; // RGBA, rows from the top (v = 1)
    // Bilinear, repeating; `srgb` turns the colour into linear light.
    Vec3 sample(Vec2 uv, bool srgb) const {
        if (w <= 0 || h <= 0) return {1, 1, 1};
        const float x = (uv.x - std::floor(uv.x)) * static_cast<float>(w) - 0.5f;
        const float y = (1.0f - (uv.y - std::floor(uv.y))) * static_cast<float>(h) - 0.5f;
        const int x0 = static_cast<int>(std::floor(x)), y0 = static_cast<int>(std::floor(y));
        const float fx = x - static_cast<float>(x0), fy = y - static_cast<float>(y0);
        auto at = [&](int xi, int yi) {
            xi = ((xi % w) + w) % w, yi = ((yi % h) + h) % h;
            const unsigned char* p = &px[(static_cast<std::size_t>(yi) * static_cast<std::size_t>(w) + static_cast<std::size_t>(xi)) * 4];
            Vec3 c{static_cast<float>(p[0]) / 255.0f, static_cast<float>(p[1]) / 255.0f, static_cast<float>(p[2]) / 255.0f};
            return srgb ? toLinear(c) : c;
        };
        return (at(x0, y0) * (1 - fx) + at(x0 + 1, y0) * fx) * (1 - fy) + (at(x0, y0 + 1) * (1 - fx) + at(x0 + 1, y0 + 1) * fx) * fy;
    }
};

struct Mat {
    Vec3 color;
    float metallic = 0, roughness = 0.45f, transmission = 0, ior = 1.5f, emission = 0;
    const Texture* colorMap = nullptr;
    const Texture* normalMap = nullptr;
    bool painted = false; // vertex colours (polypaint) over the colour
};

// A light the environment has: a disc of the sky (a soft box, the sun), sampled directly.
struct Cap {
    Vec3 dir;
    float cosMax;
    Vec3 radiance;
    float solid = 0;  // its solid angle
    float select = 0; // the chance it is the one sampled
};

struct Env {
    std::string kind;
    float light = 1;
    std::vector<Cap> caps;
    // The sky without the caps.
    Vec3 background(const Vec3& d) const {
        if (kind == "white") return Vec3{1, 1, 1} * light;
        if (kind == "sky") {
            if (d.z < 0) return Vec3{0.30f, 0.28f, 0.25f} * (0.35f * light);
            const float t = std::pow(std::max(d.z, 0.0f), 0.4f);
            return (Vec3{0.75f, 0.84f, 1.0f} * (1 - t) + Vec3{0.22f, 0.42f, 0.85f} * t) * (0.6f * light);
        }
        const float t = std::clamp((d.z + 0.2f) / 0.8f, 0.0f, 1.0f);
        const float s = t * t * (3 - 2 * t);
        if (kind == "dark") return (Vec3{0.005f, 0.005f, 0.006f} * (1 - s) + Vec3{0.03f, 0.032f, 0.036f} * s) * light;
        return (Vec3{0.025f, 0.025f, 0.027f} * (1 - s) + Vec3{0.20f, 0.205f, 0.22f} * s) * light; // studio
    }
    // The chance of choosing direction d by sampling the caps.
    float pdf(const Vec3& d) const {
        float p = 0;
        for (const Cap& c : caps)
            if (dot(d, c.dir) >= c.cosMax) p += c.select / c.solid;
        return p;
    }
    Vec3 capRadiance(const Vec3& d) const {
        Vec3 r{};
        for (const Cap& c : caps)
            if (dot(d, c.dir) >= c.cosMax) r += c.radiance;
        return r;
    }
};

Env makeEnv(const RenderSettings& s) {
    Env e;
    e.kind = s.environment, e.light = s.light;
    auto cap = [&](Vec3 dir, float halfDeg, Vec3 radiance) {
        Cap c;
        c.dir = normalize(dir);
        c.cosMax = std::cos(halfDeg * kPi / 180.0f);
        c.radiance = radiance * s.light;
        c.solid = 2.0f * kPi * (1.0f - c.cosMax);
        e.caps.push_back(c);
    };
    if (e.kind == "studio" || e.kind == "dark") {
        const float k = e.kind == "dark" ? 1.4f : 1.0f;
        cap({-0.55f, -0.75f, 0.85f}, 20.0f, Vec3{4.2f, 4.0f, 3.7f} * k); // key: warm, front left above
        cap({0.9f, -0.45f, 0.35f}, 28.0f, Vec3{1.1f, 1.15f, 1.25f} * k); // fill: cool, right
        cap({0.25f, 0.95f, 0.55f}, 14.0f, Vec3{3.6f, 3.6f, 3.6f} * k);   // rim: behind
        cap({0.0f, 0.0f, 1.0f}, 22.0f, Vec3{1.6f, 1.6f, 1.6f} * k);      // top
    } else if (e.kind == "sky") {
        const float az = s.sunAzimuth * kPi / 180.0f, el = std::clamp(s.sunElevation, 1.0f, 90.0f) * kPi / 180.0f;
        const Vec3 dir{std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
        const float half = 1.2f; // a little larger than the real sun: softer shadow edges
        const float solid = 2.0f * kPi * (1.0f - std::cos(half * kPi / 180.0f));
        cap(dir, half, Vec3{1.0f, 0.95f, 0.88f} * (8.0f / solid)); // on the ground about 4 times the sky's light, as on a clear day
    }
    float total = 0;
    for (const Cap& c : e.caps) total += lum(c.radiance) * c.solid;
    for (Cap& c : e.caps) c.select = total > 0 ? lum(c.radiance) * c.solid / total : 0;
    return e;
}

// The scene as one mesh in world coordinates, with each triangle's body.
struct World {
    MeshData mesh;
    std::vector<std::uint32_t> triBody;
    std::vector<Vec2> uv; // per corner (zeros where a body has none)
    std::vector<Mat> mats;
    std::vector<char> hasUv;
    MeshBvh bvh;
    AABB box;
    float diag = 1, eps = 1e-4f;
    bool floor = false;
    float floorZ = 0, floorR = 1;
    Vec3 centre;
    std::map<std::string, std::unique_ptr<Texture>> textures;
};

const Texture* loadTexture(World& w, const std::string& path) {
    if (path.empty()) return nullptr;
    auto& slot = w.textures[path];
    if (!slot) {
        slot = std::make_unique<Texture>();
        ImageRef img;
        img.path = path;
        if (loadImagePixels(img, nullptr)) slot->w = img.pixelWidth, slot->h = img.pixelHeight, slot->px = std::move(img.pixels);
    }
    return slot->w > 0 ? slot.get() : nullptr;
}

struct Hit {
    float t = 1e30f;
    bool floor = false;
    std::uint32_t tri = 0;
};

bool intersect(const World& w, const Ray& r, Hit& h, float tMax = 1e30f) {
    h = Hit{};
    float t = tMax;
    std::uint32_t tri = 0;
    if (!w.bvh.empty() && w.bvh.raycast(r, t, &tri)) h.t = t, h.tri = tri;
    if (w.floor && r.dir.z < -1e-6f && r.origin.z > w.floorZ) {
        const float tf = (w.floorZ - r.origin.z) / r.dir.z;
        if (tf > 0 && tf < h.t && tf < tMax) h.t = tf, h.floor = true;
    }
    return h.t < tMax && h.t < 1e29f;
}

struct Surf {
    Vec3 p, n, ng; // shading and geometric normals, facing the incoming ray
    Vec3 albedo;
    float metallic = 0, roughness = 0.5f, transmission = 0, ior = 1.5f, emission = 0;
    bool entering = true; // glass: the ray goes into the body here
};

Surf shade(World& w, const Ray& r, const Hit& h) {
    Surf s;
    s.p = r.origin + r.dir * h.t;
    if (h.floor) {
        s.n = s.ng = {0, 0, 1};
        s.albedo = {0.62f, 0.62f, 0.63f};
        s.roughness = 0.5f;
        return s;
    }
    const std::size_t k = static_cast<std::size_t>(h.tri) * 3;
    const Vertex& a = w.mesh.vertices[w.mesh.indices[k]];
    const Vertex& b = w.mesh.vertices[w.mesh.indices[k + 1]];
    const Vertex& c = w.mesh.vertices[w.mesh.indices[k + 2]];
    const Vec3 e1 = b.position - a.position, e2 = c.position - a.position;
    const Vec3 nn = cross(e1, e2);
    const float A = dot(nn, nn);
    float w1 = 0, w2 = 0;
    if (A > 0) {
        w1 = dot(cross(s.p - a.position, e2), nn) / A;
        w2 = dot(cross(e1, s.p - a.position), nn) / A;
    }
    const float w0 = 1 - w1 - w2;
    s.ng = normalize(nn);
    Vec3 n = normalize(a.normal * w0 + b.normal * w1 + c.normal * w2);
    if (!(length(n) > 0.5f)) n = s.ng;
    const std::uint32_t body = w.triBody[h.tri];
    const Mat& m = w.mats[body];
    s.albedo = m.color;
    if (m.painted) {
        auto col = [&](const Vertex& v) { return painted(v.color) ? toLinear(unpackColor(v.color)) : m.color; };
        s.albedo = col(a) * w0 + col(b) * w1 + col(c) * w2;
    }
    if (w.hasUv[body] && (m.colorMap || m.normalMap)) {
        const Vec2 ua = w.uv[k], ub = w.uv[k + 1], uc = w.uv[k + 2];
        const Vec2 uv{ua.x * w0 + ub.x * w1 + uc.x * w2, ua.y * w0 + ub.y * w1 + uc.y * w2};
        if (m.colorMap) s.albedo = mul(s.albedo, m.colorMap->sample(uv, true));
        if (m.normalMap) {
            // The face's tangent frame from its texture coordinates (as BAKE makes it).
            const float du1 = ub.x - ua.x, dv1 = ub.y - ua.y, du2 = uc.x - ua.x, dv2 = uc.y - ua.y;
            const float det = du1 * dv2 - du2 * dv1;
            if (std::fabs(det) > 1e-20f) {
                const Vec3 tu = (e1 * dv2 - e2 * dv1) / det, tv = (e2 * du1 - e1 * du2) / det;
                const Vec3 T = normalize(tu - n * dot(n, tu));
                const float hand = dot(cross(n, T), tv) < 0 ? -1.0f : 1.0f;
                const Vec3 B = cross(n, T) * hand;
                const Vec3 t = m.normalMap->sample(uv, false) * 2.0f - Vec3{1, 1, 1};
                const Vec3 pn = normalize(T * t.x + B * t.y + n * t.z);
                if (length(pn) > 0.5f) n = pn;
            }
        }
    }
    s.metallic = m.metallic, s.roughness = m.roughness, s.transmission = m.transmission, s.ior = m.ior, s.emission = m.emission;
    // Facing the ray: the side the ray came from.
    s.entering = dot(r.dir, s.ng) < 0;
    if (!s.entering) s.ng = -s.ng, n = -n;
    if (dot(n, s.ng) < 0) n = s.ng; // a shading normal turned away from the face: the face's
    s.n = n;
    return s;
}

Vec3 fresnelSchlick(const Vec3& f0, float c) {
    const float k = std::pow(1.0f - std::clamp(c, 0.0f, 1.0f), 5.0f);
    return f0 + (Vec3{1, 1, 1} - f0) * k;
}

float ggxD(float noh, float a) {
    const float a2 = a * a, d = noh * noh * (a2 - 1.0f) + 1.0f;
    return a2 / (kPi * d * d + 1e-20f);
}

float smithG1(float nox, float a) {
    const float a2 = a * a;
    return 2.0f * nox / (nox + std::sqrt(a2 + (1.0f - a2) * nox * nox) + 1e-20f);
}

float alphaOf(const Surf& s) { return std::max(s.roughness * s.roughness, 0.002f); }

Vec3 f0Of(const Surf& s) { return Vec3{0.04f, 0.04f, 0.04f} * (1 - s.metallic) + s.albedo * s.metallic; }

float specularChance(const Surf& s, float nov) {
    if (s.metallic >= 1.0f) return 1.0f;
    const float f = lum(fresnelSchlick(f0Of(s), nov));
    return std::clamp(s.metallic + (1 - s.metallic) * std::clamp(3.0f * f, 0.08f, 0.6f), 0.0f, 1.0f);
}

// The surface's reflection (times the cosine) from wi to wo, and the chance sample() picks wi.
Vec3 evalBsdf(const Surf& s, const Vec3& wo, const Vec3& wi, float& pdf) {
    pdf = 0;
    const float nol = dot(s.n, wi), nov = dot(s.n, wo);
    if (nol <= 0 || nov <= 0 || dot(s.ng, wi) <= 0) return {};
    const Vec3 h = normalize(wo + wi);
    const float noh = std::max(dot(s.n, h), 0.0f), voh = std::max(dot(wo, h), 1e-6f);
    const float a = alphaOf(s);
    const Vec3 F = fresnelSchlick(f0Of(s), voh);
    const float D = ggxD(noh, a), G = smithG1(nol, a) * smithG1(nov, a);
    const Vec3 spec = F * (D * G / (4.0f * nol * nov));
    const float kd = (1 - s.metallic) * (1 - s.transmission);
    const Vec3 diff = mul(s.albedo, Vec3{1, 1, 1} - F) * (kd / kPi);
    const float ps = specularChance(s, nov);
    pdf = ps * D * noh / (4.0f * voh) + (1 - ps) * nol / kPi;
    return (diff + spec) * nol;
}

// A new direction; the weight (reflection x cosine / chance); delta: a mirror or glass bounce.
bool sampleBsdf(const Surf& s, const Vec3& wo, Rng& rng, Vec3& wi, Vec3& weight, float& pdf, bool& delta) {
    delta = false;
    if (s.transmission > 0.5f) {
        // Smooth glass: reflect or refract by the exact Fresnel term.
        delta = true;
        const float eta = s.entering ? 1.0f / s.ior : s.ior;
        const float cosi = std::clamp(dot(wo, s.n), 0.0f, 1.0f);
        const float sin2t = eta * eta * (1.0f - cosi * cosi);
        float F = 1;
        float cost = 0;
        if (sin2t < 1) {
            cost = std::sqrt(1 - sin2t);
            const float rs = (eta * cosi - cost) / (eta * cosi + cost), rp = (cosi - eta * cost) / (cosi + eta * cost);
            F = 0.5f * (rs * rs + rp * rp);
        }
        if (rng.uni() < F) {
            wi = s.n * (2 * dot(wo, s.n)) - wo;
            weight = {1, 1, 1};
        } else {
            wi = normalize(-wo * eta + s.n * (eta * cosi - cost));
            weight = s.albedo;
        }
        pdf = 1;
        return true;
    }
    const float nov = dot(s.n, wo);
    if (nov <= 0) return false;
    const float ps = specularChance(s, nov);
    Vec3 t, b;
    frameOf(s.n, t, b);
    const float u1 = rng.uni(), u2 = rng.uni();
    if (rng.uni() < ps) {
        const float a = alphaOf(s);
        const float phi = 2 * kPi * u1;
        const float ct = std::sqrt((1 - u2) / (1 + (a * a - 1) * u2)), st = std::sqrt(std::max(0.0f, 1 - ct * ct));
        const Vec3 h = t * (st * std::cos(phi)) + b * (st * std::sin(phi)) + s.n * ct;
        wi = h * (2 * dot(wo, h)) - wo;
    } else {
        const float r = std::sqrt(u1), phi = 2 * kPi * u2;
        wi = t * (r * std::cos(phi)) + b * (r * std::sin(phi)) + s.n * std::sqrt(std::max(0.0f, 1 - u1));
    }
    const Vec3 f = evalBsdf(s, wo, wi, pdf);
    if (!(pdf > 1e-8f)) return false;
    weight = f / pdf;
    return maxc(weight) > 0;
}

float powerHeuristic(float a, float b) { return a * a / (a * a + b * b + 1e-30f); }

struct Aov {
    Vec3 albedo{1, 1, 1}, normal{};
    float depth = 1e30f;
};

Vec3 trace(World& w, const Env& env, Ray ray, Rng& rng, int maxBounces, Aov* aov) {
    Vec3 L{}, T{1, 1, 1};
    float prevPdf = 0;
    bool prevDelta = true;
    for (int depth = 0; depth <= maxBounces; ++depth) {
        Hit h;
        const bool hit = intersect(w, ray, h);
        // The floor fades into the environment far away (seen from the camera): part of the
        // pixel is the sky beyond, the rest the floor.
        if (hit && h.floor && depth == 0) {
            const Vec3 p = ray.origin + ray.dir * h.t;
            const float d = std::hypot(p.x - w.centre.x, p.y - w.centre.y);
            float f = std::clamp((d - w.floorR) / (2.0f * w.floorR), 0.0f, 1.0f);
            f = f * f * (3 - 2 * f);
            if (f > 0) {
                L += mul(T, env.background(ray.dir)) * f;
                T = T * (1 - f);
                if (f >= 1) {
                    if (aov) aov->albedo = {1, 1, 1}, aov->normal = {}, aov->depth = 1e30f;
                    break;
                }
            }
        }
        if (!hit) {
            Vec3 env1 = env.background(ray.dir);
            const Vec3 cap = depth == 0 ? Vec3{} : env.capRadiance(ray.dir); // the lights themselves stay out of the picture
            if (maxc(cap) > 0) env1 += cap * (prevDelta ? 1.0f : powerHeuristic(prevPdf, env.pdf(ray.dir)));
            L += mul(T, env1);
            if (depth == 0 && aov) aov->albedo = {1, 1, 1}, aov->normal = {}, aov->depth = 1e30f;
            break;
        }
        Surf s = shade(w, ray, h);
        if (depth == 0 && aov) aov->albedo = s.albedo, aov->normal = s.n, aov->depth = h.t;
        if (s.emission > 0) L += mul(T, s.albedo * s.emission);
        const Vec3 wo = -ray.dir;
        const Vec3 off = s.ng * w.eps;
        // Light from the environment's lights, straight (not for glass: a delta surface).
        if (s.transmission <= 0.5f && !env.caps.empty()) {
            float u = rng.uni();
            const Cap* c = &env.caps.back();
            for (const Cap& k : env.caps) {
                if (u < k.select) {
                    c = &k;
                    break;
                }
                u -= k.select;
            }
            Vec3 t, b;
            frameOf(c->dir, t, b);
            const float ct = 1 - rng.uni() * (1 - c->cosMax), st = std::sqrt(std::max(0.0f, 1 - ct * ct)), phi = 2 * kPi * rng.uni();
            const Vec3 dir = normalize(t * (st * std::cos(phi)) + b * (st * std::sin(phi)) + c->dir * ct);
            float pb = 0;
            const Vec3 f = evalBsdf(s, wo, dir, pb);
            if (maxc(f) > 0) {
                Hit sh;
                if (!intersect(w, Ray{s.p + off, dir}, sh)) {
                    const float pl = env.pdf(dir);
                    L += mul(T, mul(f, env.capRadiance(dir))) * (powerHeuristic(pl, pb) / pl);
                }
            }
        }
        if (depth == maxBounces) break;
        Vec3 wi, weight;
        float pdf = 0;
        bool delta = false;
        if (!sampleBsdf(s, wo, rng, wi, weight, pdf, delta)) break;
        T = mul(T, weight);
        prevPdf = pdf, prevDelta = delta;
        ray = Ray{s.p + (dot(wi, s.ng) > 0 ? off : -off), wi};
        if (depth >= 3) { // Russian roulette
            const float q = std::min(0.95f, maxc(T));
            if (rng.uni() >= q) break;
            T = T / q;
        }
    }
    return L;
}

// Edge-avoiding a-trous wavelet filter (5 passes of a 5x5 B3 kernel, ever wider), weighted
// by how alike the neighbours' light, normals and depths are, the light's noise estimated
// from the samples' spread.
void denoise(int W, int H, std::vector<Vec3>& illum, std::vector<float>& var, const std::vector<Vec3>& normal, const std::vector<float>& depth) {
    const float h[5] = {1.0f / 16, 1.0f / 4, 3.0f / 8, 1.0f / 4, 1.0f / 16};
    const std::size_t N = static_cast<std::size_t>(W) * static_cast<std::size_t>(H);
    std::vector<Vec3> out(N);
    std::vector<float> vout(N);
    for (int pass = 0; pass < 5; ++pass) {
        const int step = 1 << pass;
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x) {
                const std::size_t p = static_cast<std::size_t>(y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x);
                if (depth[p] >= 1e29f) { // the environment: as it is
                    out[p] = illum[p], vout[p] = var[p];
                    continue;
                }
                const float lp = lum(illum[p]);
                const float sigma = 4.0f * std::sqrt(std::max(var[p], 0.0f)) + 1e-4f;
                Vec3 sum{};
                float wsum = 0, vsum = 0;
                for (int dy = -2; dy <= 2; ++dy)
                    for (int dx = -2; dx <= 2; ++dx) {
                        const int qx = x + dx * step, qy = y + dy * step;
                        if (qx < 0 || qy < 0 || qx >= W || qy >= H) continue;
                        const std::size_t q = static_cast<std::size_t>(qy) * static_cast<std::size_t>(W) + static_cast<std::size_t>(qx);
                        if (depth[q] >= 1e29f) continue;
                        const float wn = std::pow(std::max(0.0f, dot(normal[p], normal[q])), 64.0f);
                        const float wd = std::exp(-std::fabs(depth[p] - depth[q]) / (0.01f * depth[p] * static_cast<float>(step) + 1e-6f));
                        const float wl = std::exp(-std::fabs(lp - lum(illum[q])) / sigma);
                        const float wgt = h[dx + 2] * h[dy + 2] * wn * wd * wl;
                        sum += illum[q] * wgt;
                        wsum += wgt;
                        vsum += wgt * wgt * var[q];
                    }
                out[p] = wsum > 0 ? sum / wsum : illum[p];
                vout[p] = wsum > 0 ? vsum / (wsum * wsum) : var[p];
            }
        illum.swap(out);
        var.swap(vout);
    }
}

} // namespace

bool renderImage(const Scene& scene, const RenderSettings& rs, const RenderCamera& cam, std::vector<std::uint8_t>& rgb, RenderStats* stats,
                 std::string* why, const std::function<bool(float)>& progress) {
    const auto start = std::chrono::steady_clock::now();
    World w;
    for (std::size_t i = 0; i < scene.bodies().size(); ++i) {
        const Body& b = scene.bodies()[i];
        if (!b.visible || b.mesh.indices.empty()) continue;
        MeshData m = b.mesh;
        for (Vertex& v : m.vertices) v.position += b.offset;
        const auto bodyIndex = static_cast<std::uint32_t>(w.mats.size());
        Mat mat;
        mat.color = toLinear(b.color);
        mat.metallic = b.material.metallic, mat.roughness = std::clamp(b.material.roughness, 0.0f, 1.0f);
        mat.transmission = b.material.transmission, mat.ior = std::max(b.material.ior, 1.0f), mat.emission = std::max(b.material.emission, 0.0f);
        mat.painted = std::any_of(m.vertices.begin(), m.vertices.end(), [](const Vertex& v) { return painted(v.color); });
        w.mats.push_back(mat);
        const bool mapped = m.hasUvs();
        w.hasUv.push_back(mapped ? 1 : 0);
        if (mapped) {
            w.mats.back().colorMap = loadTexture(w, b.material.colorMap);
            w.mats.back().normalMap = loadTexture(w, b.material.normalMap);
        }
        const std::size_t before = w.mesh.indices.size();
        const std::vector<Vec2> uvs = mapped ? m.uvs : std::vector<Vec2>(m.indices.size());
        m.uvs.clear();
        w.mesh.append(m);
        w.uv.insert(w.uv.end(), uvs.begin(), uvs.end());
        w.triBody.insert(w.triBody.end(), (w.mesh.indices.size() - before) / 3, bodyIndex);
    }
    if (w.mesh.indices.empty()) {
        if (why) *why = "Nothing visible to render";
        return false;
    }
    w.bvh.build(w.mesh);
    w.box = w.mesh.bounds();
    w.diag = std::max(length(w.box.max - w.box.min), 1e-3f);
    w.eps = w.diag * 1e-4f;
    w.centre = (w.box.min + w.box.max) * 0.5f;
    w.floor = rs.floor;
    w.floorZ = w.box.min.z - w.diag * 1e-4f;
    w.floorR = 0.75f * w.diag;
    const Env env = makeEnv(rs);
    const int W = std::clamp(rs.width, 8, 8192), H = std::clamp(rs.height, 8, 8192);
    const int spp = std::clamp(rs.samples, 1, 65536), bounces = std::clamp(rs.bounces, 1, 64);
    const std::size_t N = static_cast<std::size_t>(W) * static_cast<std::size_t>(H);
    std::vector<Vec3> color(N), albedo(N), normal(N);
    std::vector<float> depth(N, 1e30f), var(N, 0.0f);
    const float aspect = static_cast<float>(W) / static_cast<float>(H);
    const float th = std::tan(0.5f * cam.fovY);
    std::atomic<int> nextRow{0}, rowsDone{0};
    std::atomic<bool> stop{false};
    auto work = [&](bool caller) {
        for (int y = nextRow++; y < H && !stop; y = nextRow++) {
            for (int x = 0; x < W; ++x) {
                const std::size_t p = static_cast<std::size_t>(y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x);
                Rng rng(p * 9781u + 0x9E3779B97F4A7C15ull);
                Vec3 sum{}, alb{}, nrm{};
                float dsum = 0, l1 = 0, l2 = 0;
                int hits = 0;
                for (int sIdx = 0; sIdx < spp; ++sIdx) {
                    const float px = (static_cast<float>(x) + rng.uni()) / static_cast<float>(W) * 2 - 1;
                    const float py = 1 - (static_cast<float>(y) + rng.uni()) / static_cast<float>(H) * 2;
                    Ray r;
                    if (cam.ortho) {
                        r.origin = cam.position + cam.right * (px * 0.5f * cam.orthoHeight * aspect) + cam.up * (py * 0.5f * cam.orthoHeight);
                        r.dir = cam.forward;
                    } else {
                        r.origin = cam.position;
                        r.dir = normalize(cam.forward + cam.right * (px * th * aspect) + cam.up * (py * th));
                    }
                    Aov a;
                    Vec3 c = trace(w, env, r, rng, bounces, &a);
                    if (!(c.x == c.x) || !(c.y == c.y) || !(c.z == c.z)) c = {};
                    c = {std::min(c.x, 64.0f), std::min(c.y, 64.0f), std::min(c.z, 64.0f)}; // fireflies
                    sum += c;
                    alb += a.albedo;
                    nrm += a.normal;
                    if (a.depth < 1e29f) dsum += a.depth, ++hits;
                    const Vec3 il{c.x / std::max(a.albedo.x, 0.05f), c.y / std::max(a.albedo.y, 0.05f), c.z / std::max(a.albedo.z, 0.05f)};
                    const float li = lum(il);
                    l1 += li, l2 += li * li;
                }
                const float inv = 1.0f / static_cast<float>(spp);
                color[p] = sum * inv;
                albedo[p] = alb * inv;
                normal[p] = length(nrm) > 0 ? normalize(nrm) : Vec3{};
                depth[p] = hits * 2 > spp ? dsum / static_cast<float>(hits) : 1e30f;
                const float mean = l1 * inv;
                var[p] = std::max(0.0f, l2 * inv - mean * mean) * inv; // of the mean
            }
            ++rowsDone;
            if (caller && progress && !progress(static_cast<float>(rowsDone.load()) / static_cast<float>(H))) stop = true;
        }
    };
    const unsigned threads = std::clamp(std::thread::hardware_concurrency(), 1u, 64u);
    std::vector<std::thread> pool;
    for (unsigned i = 1; i < threads; ++i) pool.emplace_back(work, false);
    work(true);
    for (std::thread& t : pool) t.join();
    if (rs.denoise && spp < 4096) {
        // The light alone (the colours put back after): textures and paint stay sharp.
        std::vector<Vec3> illum(N);
        for (std::size_t p = 0; p < N; ++p) {
            const Vec3 a{std::max(albedo[p].x, 0.05f), std::max(albedo[p].y, 0.05f), std::max(albedo[p].z, 0.05f)};
            illum[p] = depth[p] < 1e29f ? Vec3{color[p].x / a.x, color[p].y / a.y, color[p].z / a.z} : color[p];
        }
        denoise(W, H, illum, var, normal, depth);
        for (std::size_t p = 0; p < N; ++p) {
            const Vec3 a{std::max(albedo[p].x, 0.05f), std::max(albedo[p].y, 0.05f), std::max(albedo[p].z, 0.05f)};
            if (depth[p] < 1e29f) color[p] = mul(illum[p], a);
        }
    }
    // Exposure, a filmic curve (ACES, fitted) and sRGB.
    const float gain = std::pow(2.0f, rs.exposure);
    rgb.resize(N * 3);
    auto tone = [&](float v) {
        v *= gain;
        v = (v * (2.51f * v + 0.03f)) / (v * (2.43f * v + 0.59f) + 0.14f);
        v = std::clamp(v, 0.0f, 1.0f);
        v = v <= 0.0031308f ? 12.92f * v : 1.055f * std::pow(v, 1.0f / 2.4f) - 0.055f;
        return static_cast<std::uint8_t>(std::lround(v * 255.0f));
    };
    for (std::size_t p = 0; p < N; ++p) rgb[p * 3] = tone(color[p].x), rgb[p * 3 + 1] = tone(color[p].y), rgb[p * 3 + 2] = tone(color[p].z);
    if (stats) {
        stats->triangles = w.mesh.indices.size() / 3;
        stats->seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        stats->raysPerSecond = static_cast<double>(N) * spp / std::max(stats->seconds, 1e-6);
    }
    return true;
}

} // namespace sicdm

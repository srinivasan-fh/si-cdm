#include "render/Renderer.hpp"

#include <unordered_map>
#include <thread>
#include <chrono>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>

namespace sicdm {

namespace {

// ---------------------------------------------------------------------------
// Shaders (GLSL 410 core: the OpenGL ceiling on macOS).

constexpr const char* kBackgroundVS = R"(#version 410 core
out vec2 vUV;
void main() {
    // Full-screen triangle from gl_VertexID, no vertex buffer needed.
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vUV = p;
    gl_Position = vec4(p * 2.0 - 1.0, 1.0, 1.0);
}
)";

constexpr const char* kBackgroundFS = R"(#version 410 core
in vec2 vUV;
uniform vec3 uTop;
uniform vec3 uBottom;
out vec4 fragColor;
void main() {
    float t = smoothstep(0.0, 1.0, vUV.y);
    fragColor = vec4(mix(uBottom, uTop, t), 1.0);
}
)";

constexpr const char* kMeshVS = R"(#version 410 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in float aMask;
uniform mat4 uViewProj;
uniform mat4 uModel; // body offset and simulated motion (rigid)
out vec3 vWorldPos;
out vec3 vNormal;
out float vMask;
void main() {
    vWorldPos = (uModel * vec4(aPos, 1.0)).xyz;
    vNormal = mat3(uModel) * aNormal;
    vMask = aMask;
    gl_Position = uViewProj * vec4(vWorldPos, 1.0);
}
)";

constexpr const char* kMeshFS = R"(#version 410 core
in vec3 vWorldPos;
in vec3 vNormal;
in float vMask;
uniform vec3 uColor;
uniform vec3 uCamPos;
uniform vec3 uCamRight;
uniform vec3 uCamUp;
uniform float uHighlight;
uniform vec3 uCamDir;     // view direction (used for orthographic views)
uniform float uOrtho;
uniform float uHiddenLine;
uniform vec3 uPaper;      // hidden-line face fill
uniform vec3 uInk;        // hidden-line silhouette colour
uniform float uField;     // 1: the mask is a result (0..1), drawn blue - green - red
out vec4 fragColor;
vec3 fieldColour(float t) {
    t = clamp(t, 0.0, 1.0);
    vec3 a = vec3(0.10, 0.25, 0.95), b = vec3(0.10, 0.85, 0.95), c = vec3(0.20, 0.90, 0.25), d = vec3(0.98, 0.90, 0.15), e = vec3(0.95, 0.15, 0.10);
    if (t < 0.25) return mix(a, b, t / 0.25);
    if (t < 0.5) return mix(b, c, (t - 0.25) / 0.25);
    if (t < 0.75) return mix(c, d, (t - 0.5) / 0.25);
    return mix(d, e, (t - 0.75) / 0.25);
}
void main() {
    vec3 N = normalize(vNormal);
    vec3 V = uOrtho > 0.5 ? -uCamDir : normalize(uCamPos - vWorldPos);
    if (!gl_FrontFacing) N = -N;

    if (uHiddenLine > 0.5) {
        // Faces become paper (hiding what is behind them); curved silhouettes
        // are drawn where the surface turns edge-on to the viewer.
        float ndv = abs(dot(N, V));
        float w = max(fwidth(ndv) * 2.5, 1e-4);
        float rim = 1.0 - smoothstep(0.0, w, ndv);
        vec3 ink = mix(uInk, vec3(0.35, 0.70, 1.00), uHighlight);
        vec3 paper = mix(uPaper, vec3(0.16, 0.24, 0.36), uHighlight * 0.6);
        fragColor = vec4(mix(paper, ink, rim), 1.0);
        return;
    }

    // Headlight slightly above-left of the eye plus a cool fill light.
    vec3 L1 = normalize(V + 0.35 * uCamUp - 0.25 * uCamRight);
    vec3 L2 = normalize(-V + 0.8 * uCamUp + 0.6 * uCamRight);

    float diff = max(dot(N, L1), 0.0) * 0.80 + max(dot(N, L2), 0.0) * 0.18;
    float spec = pow(max(dot(N, normalize(L1 + V)), 0.0), 64.0) * 0.35;

    // Hemisphere ambient around world +Z.
    float hemi = 0.5 + 0.5 * N.z;
    vec3 ambient = mix(vec3(0.16, 0.15, 0.14), vec3(0.24, 0.26, 0.30), hemi);

    vec3 base = mix(uColor, vec3(0.30, 0.65, 1.00), uHighlight * 0.55);
    if (uField > 1.5) { // zebra: stripes of the reflected view, smooth where the surface is
        vec3 R = reflect(-V, N);
        float f = fract(R.z * 5.0);
        float w = max(fwidth(R.z * 5.0), 1e-3);
        float stripe = smoothstep(0.5 - w, 0.5 + w, f) - smoothstep(1.0 - w, 1.0, f);
        fragColor = vec4(mix(vec3(0.04), vec3(0.96), stripe), 1.0);
        return;
    }
    if (uField > 0.5) base = mix(fieldColour(vMask), vec3(0.30, 0.65, 1.00), uHighlight * 0.3);
    else base *= 1.0 - 0.6 * clamp(vMask, 0.0, 1.0); // masked: darker
    vec3 c = base * (ambient + diff) + vec3(spec);
    // Light rim to separate silhouettes from the background.
    c += pow(1.0 - max(dot(N, V), 0.0), 3.0) * 0.08;
    fragColor = vec4(pow(c, vec3(1.0 / 1.1)), 1.0);
}
)";

constexpr const char* kLinesVS = R"(#version 410 core
layout(location = 0) in vec3 aPos;
uniform mat4 uViewProj;
uniform mat4 uModel;
void main() { gl_Position = uViewProj * uModel * vec4(aPos, 1.0); }
)";

constexpr const char* kLinesFS = R"(#version 410 core
uniform vec4 uColor;
out vec4 fragColor;
void main() { fragColor = uColor; }
)";

constexpr const char* kSketchVS = R"(#version 410 core
layout(location = 0) in vec3 aPos;
uniform mat4 uViewProj;
uniform float uPointSize;
void main() {
    gl_Position = uViewProj * vec4(aPos, 1.0);
    gl_PointSize = uPointSize;
}
)";

constexpr const char* kSketchFS = R"(#version 410 core
uniform vec4 uColor;
uniform float uRound;
out vec4 fragColor;
void main() {
    if (uRound > 0.5 && length(gl_PointCoord - vec2(0.5)) > 0.5) discard;
    fragColor = uColor;
}
)";

constexpr const char* kImageVS = R"(#version 410 core
uniform mat4 uViewProj;
uniform vec3 uCorners[4]; // bottom-left, bottom-right, top-right, top-left
out vec2 vUV;
void main() {
    const int idx[6] = int[](0, 1, 2, 0, 2, 3);
    const vec2 uv[4] = vec2[](vec2(0, 1), vec2(1, 1), vec2(1, 0), vec2(0, 0)); // pixel rows top first
    int i = idx[gl_VertexID];
    vUV = uv[i];
    gl_Position = uViewProj * vec4(uCorners[i], 1.0);
}
)";

constexpr const char* kImageFS = R"(#version 410 core
in vec2 vUV;
uniform sampler2D uTex;
uniform float uOpacity;
uniform float uHighlight;
out vec4 fragColor;
void main() {
    vec4 c = texture(uTex, vUV);
    c.rgb = mix(c.rgb, vec3(0.35, 0.65, 1.0), uHighlight * 0.18);
    fragColor = vec4(c.rgb, c.a * uOpacity);
}
)";

constexpr const char* kGridVS = R"(#version 410 core
uniform mat4 uViewProj;
uniform vec2 uCenter;   // patch centre in plane coordinates
uniform float uExtent;
uniform vec3 uAxisU;
uniform vec3 uAxisV;
uniform vec3 uOrigin;
out vec2 vPlanePos;
void main() {
    const vec2 corners[6] = vec2[](vec2(-1,-1), vec2(1,-1), vec2(1,1),
                                   vec2(-1,-1), vec2(1,1), vec2(-1,1));
    vec2 p = uCenter + corners[gl_VertexID] * uExtent;
    vPlanePos = p;
    gl_Position = uViewProj * vec4(uOrigin + uAxisU * p.x + uAxisV * p.y, 1.0);
}
)";

constexpr const char* kGridFS = R"(#version 410 core
in vec2 vPlanePos;
uniform vec2 uCenter;
uniform float uExtent;
uniform float uLevel;   // log10 of the minor spacing (fractional)
uniform vec3 uColorU;   // colour of the u axis line (v == 0)
uniform vec3 uColorV;   // colour of the v axis line (u == 0)
uniform float uStrength;
out vec4 fragColor;

float lineMask(vec2 p, float spacing, float widthPx) {
    vec2 c = p / spacing;
    vec2 d = fwidth(c);
    vec2 g = abs(fract(c - 0.5) - 0.5) / (d * widthPx);
    return 1.0 - min(min(g.x, g.y), 1.0);
}

void main() {
    vec2 p = vPlanePos;
    float lvl = floor(uLevel);
    float blend = fract(uLevel);
    float minor = pow(10.0, lvl);
    float major = minor * 10.0;

    float a = lineMask(p, minor, 1.0) * (1.0 - blend) * 0.22;
    a = max(a, lineMask(p, major, 1.2) * 0.45);
    a = max(a, lineMask(p, major * 10.0, 1.4) * 0.55);
    vec3 col = vec3(0.62, 0.66, 0.72);

    // World axes lying in the plane.
    vec2 fw = fwidth(p);
    float ax = 1.0 - min(abs(p.y) / (fw.y * 1.6), 1.0);
    float ay = 1.0 - min(abs(p.x) / (fw.x * 1.6), 1.0);
    col = mix(col, uColorU, ax);
    col = mix(col, uColorV, ay);
    a = max(a * uStrength, max(ax, ay) * 0.9);

    // Fade towards the edge of the patch so it reads as infinite.
    float r = length(p - uCenter) / uExtent;
    a *= 1.0 - smoothstep(0.35, 1.0, r);
    if (a <= 0.002) discard;
    fragColor = vec4(col, a);
}
)";

GpuMesh uploadMesh(const MeshData& m) {
    GpuMesh g;

    glGenVertexArrays(1, &g.vao);
    glBindVertexArray(g.vao);
    glGenBuffers(1, &g.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, g.vbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(m.vertices.size() * sizeof(Vertex)),
                 m.vertices.data(), GL_STATIC_DRAW);
    glGenBuffers(1, &g.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(m.indices.size() * sizeof(std::uint32_t)),
                 m.indices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<const void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<const void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<const void*>(offsetof(Vertex, mask)));
    g.indexCount = static_cast<GLsizei>(m.indices.size());

    if (!m.edgeVertices.empty()) {
        glGenVertexArrays(1, &g.edgeVao);
        glBindVertexArray(g.edgeVao);
        glGenBuffers(1, &g.edgeVbo);
        glBindBuffer(GL_ARRAY_BUFFER, g.edgeVbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(m.edgeVertices.size() * sizeof(Vec3)),
                     m.edgeVertices.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
        g.edgeVertexCount = static_cast<GLsizei>(m.edgeVertices.size());
    }

    glBindVertexArray(0);
    return g;
}

} // namespace

Renderer::Renderer()
    : m_background(kBackgroundVS, kBackgroundFS),
      m_mesh(kMeshVS, kMeshFS),
      m_lines(kLinesVS, kLinesFS),
      m_grid(kGridVS, kGridFS),
      m_sketchShader(kSketchVS, kSketchFS),
      m_imageShader(kImageVS, kImageFS) {
    glGenBuffers(1, &m_outlineVbo);
    glGenVertexArrays(1, &m_outlineVao);
    glBindVertexArray(m_outlineVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_outlineVbo);
    glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(Vec3), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
    glGenVertexArrays(1, &m_emptyVao);
    glGenVertexArrays(1, &m_sketchVao);
    glGenBuffers(1, &m_sketchVbo);
    glBindVertexArray(m_sketchVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_sketchVbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
    glBindVertexArray(0);
}

Renderer::~Renderer() {
    releaseMeshes();
    if (m_emptyVao) glDeleteVertexArrays(1, &m_emptyVao);
    if (m_sketchVao) glDeleteVertexArrays(1, &m_sketchVao);
    if (m_sketchVbo) glDeleteBuffers(1, &m_sketchVbo);
    releaseImages();
    if (m_outlineVao) glDeleteVertexArrays(1, &m_outlineVao);
    if (m_outlineVbo) glDeleteBuffers(1, &m_outlineVbo);
}

bool Renderer::valid() const noexcept {
    return m_background.valid() && m_mesh.valid() && m_lines.valid() && m_grid.valid() && m_sketchShader.valid() &&
           m_imageShader.valid();
}

namespace {

void releaseMesh(GpuMesh& g) {
    if (g.vbo) glDeleteBuffers(1, &g.vbo);
    if (g.ibo) glDeleteBuffers(1, &g.ibo);
    if (g.vao) glDeleteVertexArrays(1, &g.vao);
    if (g.edgeVbo) glDeleteBuffers(1, &g.edgeVbo);
    if (g.edgeVao) glDeleteVertexArrays(1, &g.edgeVao);
    g = GpuMesh{};
}

} // namespace

namespace {

constexpr std::size_t kLodTriangles = 60000; // shapes this big get coarser copies
constexpr std::chrono::milliseconds kLodSettle{400}; // ...once unchanged this long

// How big a coarser copy's cells may look on screen (pixels): still, so the change can't be
// seen (vertices move about half a cell), and while the view moves.
constexpr float kLodCellPixels = 2.0f, kLodCellPixelsMoving = 6.0f;

// Coarser copies of a shape: cells of 1/200, 1/80 and 1/30 of its size, each kept only
// when it cuts the triangles of the one before to 60% or fewer.
std::vector<std::pair<float, MeshData>> buildLods(const MeshData& m) {
    std::vector<std::pair<float, MeshData>> out;
    const AABB b = m.bounds();
    if (!b.valid()) return out;
    const float diag = length(b.max - b.min);
    std::size_t prev = m.indices.size() / 3;
    for (float div : {200.0f, 80.0f, 30.0f}) {
        MeshData c = clusterMesh(m, diag / div);
        const std::size_t n = c.indices.size() / 3;
        if (n == 0 || n * 10 > prev * 6) continue;
        prev = n;
        out.push_back({diag / div, std::move(c)});
    }
    return out;
}

} // namespace

void Renderer::releaseMeshes() {
    std::vector<MeshEntry*> done; // bodies with the same shape share an entry: free it once
    for (auto& e : m_entries)
        if (e && std::find(done.begin(), done.end(), e.get()) == done.end()) {
            done.push_back(e.get());
            releaseMesh(e->full);
            for (auto& [cell, g] : e->lods) releaseMesh(g);
            if (e->job.valid()) m_orphanJobs.push_back(std::move(e->job));
        }
    m_entries.clear();
    for (GpuMesh& g : m_streamed) releaseMesh(g);
    m_streamed.clear();
}

bool Renderer::pendingWork() const noexcept {
    for (const auto& e : m_entries)
        if (e && ((!e->lodStarted && e->triangles >= kLodTriangles) || e->job.valid())) return true;
    return false;
}

void Renderer::upload(const Scene& scene) {
    const auto& bodies = scene.bodies();
    // Each body's shape key (in parallel for big scenes: it reads every vertex).
    std::vector<std::uint64_t> keys(bodies.size());
    std::size_t total = 0;
    for (const Body& b : bodies) total += b.mesh.vertices.size();
    const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
    if (total > 400000 && bodies.size() > 1 && hw > 1) {
        std::atomic<std::size_t> next{0};
        std::vector<std::thread> pool;
        for (unsigned k = 0; k < std::min<unsigned>(hw, 8); ++k)
            pool.emplace_back([&] {
                for (std::size_t i; (i = next.fetch_add(1)) < bodies.size();) keys[i] = meshContentKey(bodies[i].mesh);
            });
        for (std::thread& t : pool) t.join();
    } else {
        for (std::size_t i = 0; i < bodies.size(); ++i) keys[i] = meshContentKey(bodies[i].mesh);
    }
    std::unordered_map<std::uint64_t, std::shared_ptr<MeshEntry>> old, kept;
    for (const auto& e : m_entries)
        if (e) old.emplace(e->key, e);
    std::vector<std::shared_ptr<MeshEntry>> entries;
    entries.reserve(bodies.size());
    int uploads = 0;
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const std::uint64_t key = keys[i];
        if (const auto it = kept.find(key); it != kept.end()) {
            entries.push_back(it->second);
            continue;
        }
        std::shared_ptr<MeshEntry> e;
        if (const auto it = old.find(key); it != old.end()) e = it->second;
        else {
            e = std::make_shared<MeshEntry>();
            e->key = key;
            e->full = uploadMesh(bodies[i].mesh);
            e->triangles = bodies[i].mesh.indices.size() / 3;
            e->box = bodies[i].mesh.bounds();
            e->born = std::chrono::steady_clock::now();
            ++uploads;
        }
        kept.emplace(key, e);
        entries.push_back(std::move(e));
    }
    for (auto& [key, e] : old)
        if (!kept.count(key)) {
            releaseMesh(e->full);
            for (auto& [cell, g] : e->lods) releaseMesh(g);
            if (e->job.valid()) m_orphanJobs.push_back(std::move(e->job)); // finishes on its own
        }
    m_entries = std::move(entries);
    m_stats.uploads = uploads;
    m_uploadedRevision = scene.revision();
}

void Renderer::releaseImages() {
    for (GLuint t : m_imageTextures)
        if (t) glDeleteTextures(1, &t);
    m_imageTextures.clear();
}

void Renderer::uploadImages(const Scene& scene) {
    releaseImages();
    for (const ImageRef& img : scene.images()) {
        GLuint tex = 0;
        if (!img.pixels.empty()) {
            glGenTextures(1, &tex);
            glBindTexture(GL_TEXTURE_2D, tex);
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, img.pixelWidth, img.pixelHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                         img.pixels.data());
            glGenerateMipmap(GL_TEXTURE_2D);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        }
        m_imageTextures.push_back(tex);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    m_imageRevision = scene.imageRevision();
}

void Renderer::drawImages(const Scene& scene, const OrbitCamera& cam, const ViewportSettings& s, int selected) {
    if (scene.imageRevision() != m_imageRevision) uploadImages(scene);
    const auto& images = scene.images();
    const std::size_t n = std::min(images.size(), m_imageTextures.size());
    if (n == 0) return;
    // 2D drawing window: images stay visible under the sketch for tracing.
    // 3D: pushed back in depth so bodies resting on the image plane win.
    if (s.sketchOnTop) glDisable(GL_DEPTH_TEST);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(4.0f, 8.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    m_imageShader.bind();
    m_imageShader.set("uViewProj", cam.viewProjection());
    glActiveTexture(GL_TEXTURE0);
    glUniform1i(glGetUniformLocation(m_imageShaderProgram(), "uTex"), 0);
    glBindVertexArray(m_emptyVao);
    for (std::size_t i = 0; i < n; ++i) {
        const ImageRef& img = images[i];
        if (!img.visible || !m_imageTextures[i]) continue;
        const auto c = img.corners();
        const GLint loc = glGetUniformLocation(m_imageShaderProgram(), "uCorners");
        glUniform3fv(loc, 4, &c[0].x);
        m_imageShader.set("uOpacity", img.opacity);
        m_imageShader.set("uHighlight", static_cast<int>(i) == selected ? 1.0f : 0.0f);
        glBindTexture(GL_TEXTURE_2D, m_imageTextures[i]);
        glDrawArrays(GL_TRIANGLES, 0, 6);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_POLYGON_OFFSET_FILL);
    // Outline the selected image (also shows images whose file is missing).
    for (std::size_t i = 0; i < images.size(); ++i) {
        const bool sel = static_cast<int>(i) == selected;
        const bool missing = i >= m_imageTextures.size() || !m_imageTextures[i];
        if (!images[i].visible || (!sel && !missing)) continue;
        const auto c = images[i].corners();
        glBindBuffer(GL_ARRAY_BUFFER, m_outlineVbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, 4 * sizeof(Vec3), c.data());
        m_sketchShader.bind();
        m_sketchShader.set("uViewProj", cam.viewProjection());
        m_sketchShader.set("uColor", sel ? Vec4{0.35f, 0.72f, 1.0f, 1.0f} : Vec4{0.9f, 0.4f, 0.4f, 1.0f});
        m_sketchShader.set("uRound", 0.0f);
        m_sketchShader.set("uPointSize", 1.0f);
        glBindVertexArray(m_outlineVao);
        glDrawArrays(GL_LINE_LOOP, 0, 4);
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

void Renderer::uploadSketch(const Sketch& sketch) {
    std::vector<Vec3> verts;
    m_sketchRanges.clear();
    // Construction curves dashed: dashes a fixed share of the sketch's size.
    const AABB box = sketch.bounds();
    const float dash = std::max(box.valid() ? length(box.max - box.min) / 160.0f : 1.0f, 0.05f);
    for (const SketchCurve& c : sketch.curves()) {
        const std::vector<Vec3> strip = c.worldStrip();
        if (!c.construction || c.kind == CurveKind::Point || strip.size() < 2) {
            m_sketchRanges.push_back({static_cast<GLint>(verts.size()), static_cast<GLsizei>(strip.size()), c.kind == CurveKind::Point, false});
            verts.insert(verts.end(), strip.begin(), strip.end());
            continue;
        }
        const GLint first = static_cast<GLint>(verts.size());
        const double period = 2.0 * static_cast<double>(dash);
        double phase = 0; // where in the dash-and-gap period the next piece starts
        for (std::size_t i = 1; i < strip.size(); ++i) {
            const Vec3 a = strip[i - 1], b = strip[i];
            const double l = static_cast<double>(length(b - a));
            if (!(l > 0)) continue;
            for (double t = 0; t < l;) {
                const bool drawing = phase < dash;
                // On to the end of this dash or gap (at least a sliver, so it always moves on).
                const double step = std::max(std::min(l - t, (drawing ? dash : period) - phase), 1e-9 * period);
                if (drawing) {
                    verts.push_back(a + (b - a) * static_cast<float>(t / l));
                    verts.push_back(a + (b - a) * static_cast<float>(std::min(t + step, l) / l));
                }
                t += step;
                phase += step;
                if (phase >= period) phase -= period;
            }
        }
        m_sketchRanges.push_back({first, static_cast<GLsizei>(verts.size()) - first, false, true});
    }
    glBindBuffer(GL_ARRAY_BUFFER, m_sketchVbo);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts.size() * sizeof(Vec3)), verts.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    m_sketchRevision = sketch.revision();
}

void Renderer::drawSketch(const Scene& scene, const OrbitCamera& cam, const ViewportSettings& s, int selected) {
    const auto& curves = scene.sketch().curves();
    const std::size_t n = std::min(curves.size(), m_sketchRanges.size());
    if (n == 0) return;

    // The active drawing plane shows sketches on top; in 3D they respect depth.
    if (s.sketchOnTop) glDisable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    m_sketchShader.bind();
    m_sketchShader.set("uViewProj", cam.viewProjection());
    glBindVertexArray(m_sketchVao);
    for (std::size_t i = 0; i < n; ++i) {
        if (!curves[i].visible) continue;
        const SketchRange& r = m_sketchRanges[i];
        const Vec3 c = static_cast<int>(i) == selected ? Vec3{0.35f, 0.72f, 1.0f} : i < s.sketchColors.size() ? s.sketchColors[i] : s.sketchColor;
        m_sketchShader.set("uColor", Vec4{c.x, c.y, c.z, 1.0f});
        m_sketchShader.set("uRound", r.point ? 1.0f : 0.0f);
        m_sketchShader.set("uPointSize", r.point ? 7.0f : 1.0f);
        glDrawArrays(r.point ? GL_POINTS : r.dashed ? GL_LINES : GL_LINE_STRIP, r.first, r.count);
    }
    glDisable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);
}

void Renderer::drawBackground(const ViewportSettings& s) {
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    m_background.bind();
    m_background.set("uTop", s.backgroundTop);
    m_background.set("uBottom", s.backgroundBottom);
    glBindVertexArray(m_emptyVao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}

void Renderer::drawGrid(const OrbitCamera& cam, const ViewportSettings& s) {
    // Pick a spacing that keeps ~10-100 minor cells across the view.
    const float viewSize = cam.distance() * std::tan(cam.fovY() * 0.5f) * 2.0f;
    const float level = std::log10(std::max(viewSize / 20.0f, 1e-4f));
    const float extent = std::max(cam.distance() * 8.0f, 10.0f);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    m_grid.bind();
    m_grid.set("uViewProj", cam.viewProjection());
    const PlaneFrame f = s.gridPlane == SketchPlane::Custom ? s.gridFrame : planeFrame(s.gridPlane);
    const Vec2 c = f.toPlane(cam.target());
    // Axis line colours follow the world axis they lie along (X red, Y green, Z blue).
    auto axisColor = [](const Vec3& a) {
        if (std::fabs(a.x) > 0.5f) return Vec3{0.90f, 0.30f, 0.30f};
        if (std::fabs(a.y) > 0.5f) return Vec3{0.40f, 0.80f, 0.35f};
        return Vec3{0.35f, 0.55f, 0.95f};
    };
    m_grid.set("uCenter", c);
    m_grid.set("uExtent", extent);
    m_grid.set("uLevel", level);
    m_grid.set("uOrigin", f.origin);
    m_grid.set("uAxisU", f.u);
    m_grid.set("uAxisV", f.v);
    m_grid.set("uColorU", axisColor(f.u));
    m_grid.set("uColorV", axisColor(f.v));
    m_grid.set("uStrength", s.gridStrength);
    glBindVertexArray(m_emptyVao);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void Renderer::render(const Scene& scene, const OrbitCamera& cam, const ViewportSettings& s,
                      int fbWidth, int fbHeight, int selectedBody, int selectedCurve, int selectedImage,
                      const std::vector<Mat4>* motion, const std::vector<const MeshData*>* overrides) {
    glViewport(0, 0, fbWidth, fbHeight);
    glClearColor(s.backgroundBottom.x, s.backgroundBottom.y, s.backgroundBottom.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_MULTISAMPLE);

    drawBackground(s);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE); // CAD bodies may be open; light both sides.

    const Mat4 vp = cam.viewProjection();
    const auto& bodies = scene.bodies();
    const std::size_t count = std::min(bodies.size(), m_entries.size());
    ++m_frame;
    // Coarser copies: finished ones uploaded, new ones started for big shapes that have
    // stopped changing; jobs of shapes since replaced let go once done.
    std::erase_if(m_orphanJobs, [](auto& f) { return !f.valid() || f.wait_for(std::chrono::seconds(0)) == std::future_status::ready; });
    for (std::size_t i = 0; i < count; ++i) {
        MeshEntry& e = *m_entries[i];
        if (!e.lodStarted && e.triangles >= kLodTriangles && std::chrono::steady_clock::now() - e.born >= kLodSettle) {
            e.lodStarted = true;
            e.job = std::async(std::launch::async, [m = bodies[i].mesh]() { return buildLods(m); });
        }
        if (e.job.valid() && e.job.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            for (auto& [cell, mesh] : e.job.get()) e.lods.push_back({cell, uploadMesh(mesh)});
        }
    }
    // Shapes that change while simulating (chains / belts moving along their loop)
    // are streamed each frame in place of the uploaded mesh.
    for (std::size_t i = count; i < m_streamed.size(); ++i) releaseMesh(m_streamed[i]); // bodies gone: free their buffers first
    m_streamed.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        releaseMesh(m_streamed[i]);
        if (overrides && i < overrides->size() && (*overrides)[i]) m_streamed[i] = uploadMesh(*(*overrides)[i]);
    }
    auto model = [&](std::size_t i) {
        const Mat4 at = translate(bodies[i].offset);
        return motion && i < motion->size() ? (*motion)[i] * at : at;
    };
    // What each body is drawn from: its streamed shape, or the coarsest copy whose cells
    // come out under a pixel at its size on screen, or its own shape.
    std::vector<const GpuMesh*> drawn(count, nullptr);
    m_stats.triangles = m_stats.fullTriangles = 0;
    m_stats.coarseBodies = 0;
    // The view moving (orbit, pan, zoom): coarser copies may be drawn, and once it stops the
    // next frame draws them fine again.
    bool moving = false;
    for (int c = 0; c < 4 && !moving; ++c)
        for (int r = 0; r < 4 && !moving; ++r) moving = std::fabs(vp.m[c][r] - m_lastViewProj.m[c][r]) > 1e-7f * (1.0f + std::fabs(vp.m[c][r]));
    m_lastViewProj = vp;
    const float cellLimit = moving ? kLodCellPixelsMoving : kLodCellPixels;
    m_refine = false;
    for (std::size_t i = 0; i < count; ++i) {
        const MeshEntry& e = *m_entries[i];
        drawn[i] = m_streamed[i].vao ? &m_streamed[i] : &e.full;
        if (!bodies[i].visible) continue;
        m_stats.fullTriangles += e.triangles;
        if (!m_streamed[i].vao && !e.lods.empty() && e.box.valid()) {
            const Mat4 mdl = model(i);
            const Vec3 c = mdl.transformPoint((e.box.min + e.box.max) * 0.5f);
            const float R = 0.5f * length(e.box.max - e.box.min);
            const Vec4 c0 = vp * Vec4{c.x, c.y, c.z, 1.0f};
            const Vec3 up = c + cam.up() * R;
            const Vec4 c1 = vp * Vec4{up.x, up.y, up.z, 1.0f};
            if (c0.w > 1e-6f && c1.w > 1e-6f && R > 0) {
                const float px = std::hypot(c1.x / c1.w - c0.x / c0.w, c1.y / c1.w - c0.y / c0.w) * 0.5f * static_cast<float>(fbHeight);
                const float perUnit = px / R; // pixels per mesh unit
                for (const auto& [cell, g] : e.lods) {
                    if (cell * perUnit > cellLimit) break;
                    drawn[i] = &g;
                    if (cell * perUnit > kLodCellPixels) m_refine = true; // coarser than a still view takes
                }
                if (drawn[i] != &e.full) ++m_stats.coarseBodies;
            }
        }
        m_stats.triangles += static_cast<std::size_t>(drawn[i]->indexCount) / 3;
    }
    auto gpu = [&](std::size_t i) -> const GpuMesh& { return *drawn[i]; };

    // Solid pass, pushed back slightly so edges win the depth test.
    const bool hidden = s.displayMode == DisplayMode::HiddenLine;
    if (s.displayMode != DisplayMode::Wireframe) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0f, 1.0f);
        m_mesh.bind();
        m_mesh.set("uViewProj", vp);
        m_mesh.set("uHiddenLine", hidden ? 1.0f : 0.0f);
        m_mesh.set("uPaper", s.backgroundBottom);
        m_mesh.set("uInk", s.edgeColor);
        m_mesh.set("uOrtho", cam.projection() == Projection::Orthographic ? 1.0f : 0.0f);
        m_mesh.set("uCamDir", cam.forward());
        m_mesh.set("uCamPos", cam.position());
        m_mesh.set("uCamRight", cam.right());
        m_mesh.set("uCamUp", cam.up());
        for (std::size_t i = 0; i < count; ++i) {
            if (!bodies[i].visible) continue;
            m_mesh.set("uColor", bodies[i].color);
            m_mesh.set("uField", bodies[i].field.empty() ? 0.0f : bodies[i].field.rfind("zebra", 0) == 0 ? 2.0f : 1.0f);
            m_mesh.set("uModel", model(i));
            m_mesh.set("uHighlight", static_cast<int>(i) == selectedBody ? 1.0f : 0.0f);
            glBindVertexArray(gpu(i).vao);
            glDrawElements(GL_TRIANGLES, gpu(i).indexCount, GL_UNSIGNED_INT, nullptr);
        }
        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    // Edge / wireframe pass.
    if (s.displayMode != DisplayMode::Shaded) {
        m_lines.bind();
        m_lines.set("uViewProj", vp);
        const bool wire = s.displayMode == DisplayMode::Wireframe;
        for (std::size_t i = 0; i < count; ++i) {
            if (!bodies[i].visible) continue;
            const bool sel = static_cast<int>(i) == selectedBody;
            m_lines.set("uModel", model(i));
            if (wire) {
                const Vec3 c = sel ? Vec3{0.35f, 0.70f, 1.0f} : bodies[i].color;
                m_lines.set("uColor", Vec4{c.x, c.y, c.z, 1.0f});
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glBindVertexArray(gpu(i).vao);
                glDrawElements(GL_TRIANGLES, gpu(i).indexCount, GL_UNSIGNED_INT, nullptr);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            } else if (const GpuMesh& em = m_streamed[i].vao ? m_streamed[i] : m_entries[i]->full; em.edgeVao) { // a coarser copy keeps the full edges
                const Vec3 c = sel ? (hidden ? Vec3{0.35f, 0.70f, 1.0f} : Vec3{0.10f, 0.45f, 0.95f}) : s.edgeColor;
                m_lines.set("uColor", Vec4{c.x, c.y, c.z, 1.0f});
                glBindVertexArray(em.edgeVao);
                glDrawArrays(GL_LINES, 0, em.edgeVertexCount);
            }
        }
    }

    if (s.showGrid) drawGrid(cam, s);
    drawImages(scene, cam, s, selectedImage);
    if (scene.sketch().revision() != m_sketchRevision) uploadSketch(scene.sketch());
    drawSketch(scene, cam, s, selectedCurve);

    glBindVertexArray(0);
    glUseProgram(0);
}

} // namespace sicdm

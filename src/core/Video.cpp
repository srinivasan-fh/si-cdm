// Turntable and animation videos: frames from the path tracer, written as MJPEG AVI, APNG,
// GIF or numbered PNG frames.

#include "core/Video.hpp"

#include "core/FileIO.hpp"
#include "core/ImageWrite.hpp"
#include "core/Motion.hpp"
#include "core/Physics.hpp"
#include "core/Scene.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>

namespace sicdm {

namespace {

constexpr float kHalfTurn = 3.14159265358979f; // pi

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

void le16(std::string& s, std::uint32_t v) { s += static_cast<char>(v & 0xFF), s += static_cast<char>((v >> 8) & 0xFF); }
void le32(std::string& s, std::uint32_t v) { le16(s, v & 0xFFFF), le16(s, v >> 16); }
void be16(std::string& s, std::uint32_t v) { s += static_cast<char>((v >> 8) & 0xFF), s += static_cast<char>(v & 0xFF); }
void be32(std::string& s, std::uint32_t v) { be16(s, v >> 16), be16(s, v & 0xFFFF); }
std::uint32_t readBe32(const std::string& s, std::size_t at) {
    return static_cast<std::uint32_t>(static_cast<unsigned char>(s[at])) << 24 | static_cast<std::uint32_t>(static_cast<unsigned char>(s[at + 1])) << 16 |
           static_cast<std::uint32_t>(static_cast<unsigned char>(s[at + 2])) << 8 | static_cast<std::uint32_t>(static_cast<unsigned char>(s[at + 3]));
}

// A RIFF chunk: its id, size and data, padded to an even length.
std::string riffChunk(const char id[4], const std::string& data) {
    std::string s(id, 4);
    le32(s, static_cast<std::uint32_t>(data.size()));
    s += data;
    if (data.size() % 2) s += '\0';
    return s;
}

std::string riffList(const char type[4], const std::string& body) { return riffChunk("LIST", std::string(type, 4) + body); }

// --- APNG ---------------------------------------------------------------------------------

class ApngBuilder {
public:
    ApngBuilder(int w, int h, int fps) : m_w(w), m_h(h), m_fps(fps) {}

    void add(const std::vector<std::uint8_t>& rgb) {
        const std::string png = encodePngCompressed(m_w, m_h, 3, rgb);
        // Its IHDR (the first frame's is the file's) and IDAT chunks.
        std::string control;
        be32(control, m_seq++);
        be32(control, static_cast<std::uint32_t>(m_w)), be32(control, static_cast<std::uint32_t>(m_h));
        be32(control, 0), be32(control, 0);
        be16(control, 1), be16(control, static_cast<std::uint32_t>(m_fps)); // a frame lasts 1/fps s
        control += '\0', control += '\0';                                     // dispose: none; blend: source
        m_body += pngChunk("fcTL", control);
        for (std::size_t at = 8; at + 12 <= png.size();) {
            const std::uint32_t len = readBe32(png, at);
            const std::string type = png.substr(at + 4, 4);
            const std::string data = png.substr(at + 8, len);
            if (type == "IHDR" && m_frames == 0) m_ihdr = data;
            if (type == "IDAT") {
                if (m_frames == 0) {
                    m_body += pngChunk("IDAT", data);
                } else {
                    std::string d;
                    be32(d, m_seq++);
                    m_body += pngChunk("fdAT", d + data);
                }
            }
            at += 12 + len;
        }
        ++m_frames;
    }

    std::string finish() const {
        std::string out("\x89PNG\r\n\x1a\n", 8);
        out += pngChunk("IHDR", m_ihdr);
        std::string actl;
        be32(actl, static_cast<std::uint32_t>(m_frames)), be32(actl, 0); // played forever
        out += pngChunk("acTL", actl);
        out += m_body;
        out += pngChunk("IEND", "");
        return out;
    }

private:
    int m_w, m_h, m_fps;
    std::uint32_t m_seq = 0;
    int m_frames = 0;
    std::string m_ihdr, m_body;
};

// --- GIF ----------------------------------------------------------------------------------

using Rgb = std::array<std::uint8_t, 3>;

// 256 colours for all the frames: the colours sampled from them split at the median of
// their widest channel, box by box (median cut), each box's mean a colour.
std::vector<Rgb> medianCutPalette(const std::vector<std::vector<std::uint8_t>>& frames, std::size_t pixels) {
    std::vector<Rgb> samples;
    const std::size_t pick = std::min<std::size_t>(frames.size(), 8);
    const std::size_t step = std::max<std::size_t>(1, pixels * pick / 200000);
    for (std::size_t k = 0; k < pick; ++k) {
        const auto& f = frames[k * frames.size() / pick];
        for (std::size_t p = (k * 7) % step; p < pixels; p += step) samples.push_back({f[3 * p], f[3 * p + 1], f[3 * p + 2]});
    }
    struct Box {
        std::size_t begin, end;
        int channel = 0, range = 0;
    };
    auto measure = [&](Box& b) {
        int lo[3] = {255, 255, 255}, hi[3] = {0, 0, 0};
        for (std::size_t i = b.begin; i < b.end; ++i)
            for (int c = 0; c < 3; ++c) lo[c] = std::min<int>(lo[c], samples[i][static_cast<std::size_t>(c)]), hi[c] = std::max<int>(hi[c], samples[i][static_cast<std::size_t>(c)]);
        b.channel = 0, b.range = -1;
        for (int c = 0; c < 3; ++c)
            if (hi[c] - lo[c] > b.range) b.range = hi[c] - lo[c], b.channel = c;
    };
    std::vector<Box> boxes{{0, samples.size()}};
    measure(boxes[0]);
    while (boxes.size() < 256) {
        // The box to split: the most spread out, weighted by how many it holds.
        std::size_t best = boxes.size();
        double score = 0;
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            const double s = static_cast<double>(boxes[i].range) * std::sqrt(static_cast<double>(boxes[i].end - boxes[i].begin));
            if (boxes[i].end - boxes[i].begin > 1 && boxes[i].range > 0 && s > score) score = s, best = i;
        }
        if (best == boxes.size()) break;
        Box b = boxes[best];
        const auto c = static_cast<std::size_t>(b.channel);
        const auto first = samples.begin() + static_cast<std::ptrdiff_t>(b.begin), last = samples.begin() + static_cast<std::ptrdiff_t>(b.end);
        auto mid = first + (last - first) / 2;
        std::nth_element(first, mid, last, [c](const Rgb& x, const Rgb& y) { return x[c] < y[c]; });
        // Split by value at the median, so one colour never lands in both boxes (its
        // samples would pull both means off it).
        const std::uint8_t m = (*mid)[c];
        mid = std::partition(first, last, [c, m](const Rgb& x) { return x[c] < m; });
        if (mid == first) mid = std::partition(first, last, [c, m](const Rgb& x) { return x[c] <= m; });
        const auto split = b.begin + static_cast<std::size_t>(mid - first);
        Box l{b.begin, split}, r{split, b.end};
        measure(l), measure(r);
        boxes[best] = l;
        boxes.push_back(r);
    }
    std::vector<Rgb> palette;
    for (const Box& b : boxes) {
        double s[3] = {0, 0, 0};
        for (std::size_t i = b.begin; i < b.end; ++i)
            for (int c = 0; c < 3; ++c) s[c] += samples[i][static_cast<std::size_t>(c)];
        const double n = static_cast<double>(std::max<std::size_t>(b.end - b.begin, 1));
        palette.push_back({static_cast<std::uint8_t>(std::lround(s[0] / n)), static_cast<std::uint8_t>(std::lround(s[1] / n)), static_cast<std::uint8_t>(std::lround(s[2] / n))});
    }
    while (palette.size() < 256) palette.push_back({0, 0, 0});
    return palette;
}

// LZW codes, least significant bit first, in sub-blocks of at most 255 bytes.
class GifLzw {
public:
    explicit GifLzw(std::string& out) : m_out(out) {}

    void encode(const std::vector<std::uint8_t>& indices) {
        m_out += '\x08'; // the minimum code size: 8-bit indices
        reset();
        put(kClear);
        int prefix = -1;
        for (std::uint8_t k : indices) {
            if (prefix < 0) {
                prefix = k;
                continue;
            }
            const std::uint32_t key = static_cast<std::uint32_t>(prefix) << 8 | k;
            const int found = find(key);
            if (found >= 0) {
                prefix = found;
                continue;
            }
            put(static_cast<std::uint32_t>(prefix));
            add(key);
            prefix = k;
        }
        if (prefix >= 0) {
            put(static_cast<std::uint32_t>(prefix));
            // The decoder adds an entry on that last code: its code size may grow first.
            if (m_next < 4096 && ++m_next > (1 << m_size) && m_size < 12) ++m_size;
        }
        put(kEnd);
        if (m_bits) byte(static_cast<std::uint8_t>(m_acc));
        flushBlock();
        m_out += '\0'; // no more blocks
    }

private:
    static constexpr std::uint32_t kClear = 256, kEnd = 257;
    static constexpr std::size_t kTable = 8192; // open addressing, more than twice the codes
    std::string& m_out;
    std::array<std::uint32_t, kTable> m_keys{};
    std::array<std::int16_t, kTable> m_codes{};
    int m_next = 258, m_size = 9;
    std::uint32_t m_acc = 0;
    int m_bits = 0;
    std::string m_block;

    void reset() {
        m_codes.fill(-1);
        m_next = 258, m_size = 9;
    }
    std::size_t slot(std::uint32_t key) const { return (key * 2654435761u >> 19) & (kTable - 1); }
    int find(std::uint32_t key) const {
        for (std::size_t s = slot(key);; s = (s + 1) & (kTable - 1)) {
            if (m_codes[s] < 0) return -1;
            if (m_keys[s] == key) return m_codes[s];
        }
    }
    void add(std::uint32_t key) {
        if (m_next >= 4096) return;
        std::size_t s = slot(key);
        while (m_codes[s] >= 0) s = (s + 1) & (kTable - 1);
        m_keys[s] = key, m_codes[s] = static_cast<std::int16_t>(m_next++);
        if (m_next > (1 << m_size) && m_size < 12) ++m_size;
        if (m_next == 4096) { // full: start again
            put(kClear);
            reset();
        }
    }
    void put(std::uint32_t code) {
        m_acc |= code << m_bits;
        m_bits += m_size;
        while (m_bits >= 8) {
            byte(static_cast<std::uint8_t>(m_acc & 0xFF));
            m_acc >>= 8, m_bits -= 8;
        }
    }
    void byte(std::uint8_t b) {
        m_block += static_cast<char>(b);
        if (m_block.size() == 255) flushBlock();
    }
    void flushBlock() {
        if (m_block.empty()) return;
        m_out += static_cast<char>(m_block.size());
        m_out += m_block;
        m_block.clear();
    }
};

// --- the camera and the frames ------------------------------------------------------------

RenderCamera orbitCamera(const Vec3& centre, float radius, float fovY, float aspect, float azimuth, float elevation) {
    const float fovX = 2 * std::atan(std::tan(fovY / 2) * aspect);
    const float distance = radius / std::sin(0.5f * std::min(fovY, fovX)) * 1.04f;
    const Vec3 dir{std::cos(elevation) * std::cos(azimuth), std::cos(elevation) * std::sin(azimuth), std::sin(elevation)};
    RenderCamera c;
    c.position = centre + dir * distance;
    c.forward = dir * -1.0f;
    c.right = normalize(cross(c.forward, Vec3{0, 0, 1}));
    c.up = cross(c.right, c.forward);
    c.fovY = fovY;
    return c;
}

// The visible bodies' bounds, placed by the transforms.
AABB placedBounds(const Scene& scene, const std::vector<Mat4>* tf) {
    AABB box;
    for (std::size_t i = 0; i < scene.bodies().size(); ++i) {
        const Body& b = scene.bodies()[i];
        if (!b.visible || b.mesh.indices.empty()) continue;
        const AABB bb = b.bounds();
        for (int k = 0; k < 8; ++k) {
            const Vec3 p{k & 1 ? bb.max.x : bb.min.x, k & 2 ? bb.max.y : bb.min.y, k & 4 ? bb.max.z : bb.min.z};
            box.expand(tf && i < tf->size() ? (*tf)[i].transformPoint(p) : p);
        }
    }
    return box;
}

} // namespace

bool videoFormatOf(const std::string& path, std::string& format, std::string* why) {
    const std::size_t slash = path.find_last_of("/\\"), dot = path.find_last_of('.');
    if (path.empty()) {
        if (why) *why = "Give the video's file: .avi, .png (animated), .gif, or a folder for the frames";
        return false;
    }
    if (slash == path.size() - 1 || dot == std::string::npos || (slash != std::string::npos && dot < slash)) {
        format = "frames";
        return true;
    }
    const std::string ext = lowerOf(path.substr(dot));
    if (ext == ".avi") format = "avi";
    else if (ext == ".png" || ext == ".apng") format = "apng";
    else if (ext == ".gif") format = "gif";
    else {
        if (why)
            *why = "Videos are written as .avi (MJPEG), .png (animated PNG), .gif or a folder of frames" +
                   std::string(ext == ".mp4" || ext == ".mov" || ext == ".webm" ? " (make an MP4 from the AVI: ffmpeg -i video.avi video.mp4)" : "");
        return false;
    }
    return true;
}

std::string encodeAviMjpeg(const std::vector<std::string>& jpegs, int width, int height, int fps) {
    std::size_t largest = 0;
    for (const std::string& j : jpegs) largest = std::max(largest, j.size());
    const auto n = static_cast<std::uint32_t>(jpegs.size());
    std::string avih; // the main header
    le32(avih, static_cast<std::uint32_t>(1000000 / std::max(fps, 1)));
    le32(avih, static_cast<std::uint32_t>(largest * static_cast<std::size_t>(fps)));
    le32(avih, 0);
    le32(avih, 0x10); // it has an index
    le32(avih, n);
    le32(avih, 0);
    le32(avih, 1); // one stream
    le32(avih, static_cast<std::uint32_t>(largest + 8));
    le32(avih, static_cast<std::uint32_t>(width)), le32(avih, static_cast<std::uint32_t>(height));
    for (int i = 0; i < 4; ++i) le32(avih, 0);
    std::string strh; // the video stream
    strh += "vidsMJPG";
    le32(strh, 0), le16(strh, 0), le16(strh, 0), le32(strh, 0);
    le32(strh, 1), le32(strh, static_cast<std::uint32_t>(fps)); // scale, rate: fps frames a second
    le32(strh, 0), le32(strh, n);
    le32(strh, static_cast<std::uint32_t>(largest + 8));
    le32(strh, 0xFFFFFFFFu), le32(strh, 0);
    le16(strh, 0), le16(strh, 0), le16(strh, static_cast<std::uint32_t>(width)), le16(strh, static_cast<std::uint32_t>(height));
    std::string strf; // its format: BITMAPINFOHEADER
    le32(strf, 40), le32(strf, static_cast<std::uint32_t>(width)), le32(strf, static_cast<std::uint32_t>(height));
    le16(strf, 1), le16(strf, 24);
    strf += "MJPG";
    le32(strf, static_cast<std::uint32_t>(width * height * 3));
    for (int i = 0; i < 4; ++i) le32(strf, 0);
    const std::string hdrl = riffList("hdrl", riffChunk("avih", avih) + riffList("strl", riffChunk("strh", strh) + riffChunk("strf", strf)));
    std::string movi = "movi", idx;
    for (const std::string& j : jpegs) {
        idx += "00dc";
        le32(idx, 0x10); // a key frame
        le32(idx, static_cast<std::uint32_t>(movi.size())); // from the 'movi' list's type
        le32(idx, static_cast<std::uint32_t>(j.size()));
        movi += riffChunk("00dc", j);
    }
    return riffChunk("RIFF", "AVI " + hdrl + riffChunk("LIST", movi) + riffChunk("idx1", idx));
}

std::string encodeApng(const std::vector<std::vector<std::uint8_t>>& frames, int width, int height, int fps) {
    ApngBuilder b(width, height, fps);
    for (const auto& f : frames) b.add(f);
    return b.finish();
}

std::string encodeGif(const std::vector<std::vector<std::uint8_t>>& frames, int width, int height, int fps) {
    const std::size_t pixels = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    const std::vector<Rgb> palette = medianCutPalette(frames, pixels);
    // The nearest palette colour of each 5-bit-per-channel colour, found when first needed.
    std::vector<std::int16_t> nearest(32 * 32 * 32, -1);
    auto lookup = [&](int r, int g, int b) {
        const std::size_t k = static_cast<std::size_t>((r >> 3) << 10 | (g >> 3) << 5 | (b >> 3));
        if (nearest[k] < 0) {
            const int cr = (r & ~7) + 4, cg = (g & ~7) + 4, cb = (b & ~7) + 4;
            int best = 0, bestD = 1 << 30;
            for (std::size_t i = 0; i < palette.size(); ++i) {
                const int dr = cr - palette[i][0], dg = cg - palette[i][1], db = cb - palette[i][2];
                const int d = 2 * dr * dr + 4 * dg * dg + 3 * db * db;
                if (d < bestD) bestD = d, best = static_cast<int>(i);
            }
            nearest[k] = static_cast<std::int16_t>(best);
        }
        return static_cast<std::uint8_t>(nearest[k]);
    };
    // Ordered dithering (a Bayer matrix): the same pattern every frame, so it doesn't crawl.
    static const int kBayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
    std::string out = "GIF89a";
    le16(out, static_cast<std::uint32_t>(width)), le16(out, static_cast<std::uint32_t>(height));
    out += '\xF7'; // a global colour table of 256, 8 bits a channel
    out += '\0', out += '\0';
    for (const Rgb& c : palette) out += static_cast<char>(c[0]), out += static_cast<char>(c[1]), out += static_cast<char>(c[2]);
    out += "\x21\xFF\x0BNETSCAPE2.0\x03\x01";
    le16(out, 0); // loops forever
    out += '\0';
    const int delay = std::max(2, static_cast<int>(std::lround(100.0 / std::max(fps, 1))));
    std::vector<std::uint8_t> idx(pixels);
    for (const auto& f : frames) {
        out += "\x21\xF9\x04";
        out += '\x04'; // keep the frame (no transparency)
        le16(out, static_cast<std::uint32_t>(delay));
        out += '\0', out += '\0';
        out += '\x2C';
        le16(out, 0), le16(out, 0), le16(out, static_cast<std::uint32_t>(width)), le16(out, static_cast<std::uint32_t>(height));
        out += '\0';
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x) {
                const std::size_t p = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
                const int d = (kBayer[y & 3][x & 3] - 8) * 3 / 2; // about +-12
                idx[p] = lookup(std::clamp(f[3 * p] + d, 0, 255), std::clamp(f[3 * p + 1] + d, 0, 255), std::clamp(f[3 * p + 2] + d, 0, 255));
            }
        GifLzw(out).encode(idx);
    }
    out += '\x3B';
    return out;
}

bool makeVideo(const Scene& scene, const VideoRequest& rq, VideoReport* report, std::string* why, const std::function<bool(int, int)>& progress) {
    VideoReport local;
    VideoReport& rep = report ? *report : local;
    rep = VideoReport{};
    auto fail = [&](const std::string& w) {
        if (why) *why = w;
        return false;
    };
    std::string format;
    if (!videoFormatOf(rq.path, format, why)) return false;
    RenderSettings rs = rq.render;
    rs.width = std::clamp(rs.width, 16, 4096), rs.height = std::clamp(rs.height, 16, 4096);
    if (format == "avi") rs.width += rs.width % 2, rs.height += rs.height % 2; // players want even sizes
    const int fps = std::clamp(rq.fps, 1, 120);
    const double seconds = rq.seconds > 0 ? rq.seconds : rq.animate ? static_cast<double>(suggestedDuration(scene)) : 6.0;
    const int n = std::max(1, static_cast<int>(std::lround(seconds * fps)));
    if (n > 20000) return fail("Too many frames (at most 20000): fewer SECONDS or a lower FPS");
    const float turns = rq.animate ? rq.turns : (rq.turns != 0 ? rq.turns : 1.0f);
    const float aspect = static_cast<float>(rs.width) / static_cast<float>(rs.height);

    // Where the bodies are in each frame, and the space they take over the whole video.
    std::vector<std::vector<Mat4>> poses;
    AABB extent;
    if (rq.animate) {
        poses.reserve(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            poses.push_back(simulationTransforms(scene, static_cast<float>(rq.from + static_cast<double>(i) / fps)));
            const AABB b = placedBounds(scene, &poses.back());
            if (b.valid()) extent.expand(b.min), extent.expand(b.max);
        }
    } else {
        extent = placedBounds(scene, nullptr);
    }
    if (!extent.valid()) return fail("Nothing visible to film");

    // The cameras: going round the model, or still.
    OrbitCamera fitted;
    fitted.setAspect(aspect);
    fitted.setStandardView(rq.view.value_or(StandardView::Isometric), false);
    fitted.fitToBounds(extent, false);
    const RenderCamera still = rq.camera && turns == 0 ? *rq.camera : renderCamera(fitted);
    const Vec3 centre = (extent.min + extent.max) * 0.5f;
    const float radius = std::max(0.5f * length(extent.max - extent.min), 1e-3f);
    float startAzimuth = -0.25f * kHalfTurn; // as the isometric view
    {
        const Vec3 back = (rq.camera ? rq.camera->forward : fitted.forward()) * -1.0f;
        if (std::fabs(back.x) + std::fabs(back.y) > 1e-6f) startAzimuth = std::atan2(back.y, back.x);
    }
    const float elevation = std::clamp(rq.elevation, -85.0f, 85.0f) * kHalfTurn / 180.0f;

    rep.width = rs.width, rep.height = rs.height, rep.fps = fps;
    std::vector<std::string> jpegs;
    std::vector<std::vector<std::uint8_t>> raw;
    ApngBuilder apng(rs.width, rs.height, fps);
    std::filesystem::path folder;
    if (format == "frames") {
        folder = std::filesystem::path(rq.path);
        std::error_code ec;
        std::filesystem::create_directories(folder, ec);
        if (ec) return fail("Cannot make the folder " + rq.path + ": " + ec.message());
    }
    const auto start = std::chrono::steady_clock::now();
    int made = 0;
    for (int i = 0; i < n; ++i) {
        const RenderCamera cam = turns != 0 ? orbitCamera(centre, radius, still.fovY, aspect, startAzimuth + 2 * kHalfTurn * turns * static_cast<float>(i) / static_cast<float>(n), elevation)
                                       : still;
        RenderPose pose;
        pose.transforms = rq.animate ? &poses[static_cast<std::size_t>(i)] : nullptr;
        pose.extent = &extent;
        std::vector<std::uint8_t> px;
        std::string w;
        if (!renderImage(scene, rs, cam, px, nullptr, &w, {}, pose)) return fail("Cannot render frame " + std::to_string(i + 1) + ": " + w);
        if (format == "avi") jpegs.push_back(encodeJpeg(rs.width, rs.height, px, rq.quality));
        else if (format == "apng") apng.add(px);
        else if (format == "gif") raw.push_back(std::move(px));
        else {
            char name[32];
            std::snprintf(name, sizeof name, "frame_%05d.png", i + 1);
            std::string err;
            if (!writeFile((folder / name).string(), encodePngCompressed(rs.width, rs.height, 3, px), &err)) return fail(err);
        }
        ++made;
        if (progress && !progress(made, n)) {
            rep.stopped = true;
            break;
        }
    }
    rep.renderSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    rep.frames = made;
    rep.seconds = static_cast<double>(made) / fps;
    std::string bytes;
    if (format == "avi") bytes = encodeAviMjpeg(jpegs, rs.width, rs.height, fps), rep.format = "MJPEG AVI";
    else if (format == "apng") bytes = apng.finish(), rep.format = "animated PNG";
    else if (format == "gif") {
        bytes = encodeGif(raw, rs.width, rs.height, fps), rep.format = "animated GIF";
        rep.fps = 100.0 / std::max(2, static_cast<int>(std::lround(100.0 / fps))); // GIF counts hundredths of a second
    } else {
        rep.format = "PNG frames";
        return true;
    }
    rep.bytes = bytes.size();
    return writeFile(rq.path, bytes, why);
}

std::string videoSummary(const VideoReport& r, const std::string& path) {
    char b[256];
    std::snprintf(b, sizeof b, "%d frames, %d x %d at %.4g fps (%.1f s of video), rendered in %.1f s", r.frames, r.width, r.height, r.fps, r.seconds,
                  r.renderSeconds);
    std::string s = std::string(r.stopped ? "Stopped: " : "Made ") + b + " -> " + path + " (" + r.format;
    if (r.bytes) s += ", " + std::to_string((r.bytes + 512) / 1024) + " KB";
    return s + ")";
}

} // namespace sicdm

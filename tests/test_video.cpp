#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/ImageWrite.hpp"
#include "core/Json.hpp"
#include "core/Mcp.hpp"
#include "core/Scene.hpp"
#include "core/Video.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

std::string tempPath(const std::string& name) { return (std::filesystem::temp_directory_path() / name).string(); }

std::string readAll(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream s;
    s << f.rdbuf();
    return s.str();
}

// Decodes an image file (stb_image: PNG, JPEG, the first frame of a GIF) to RGB.
std::vector<std::uint8_t> decode(const std::string& bytes, const std::string& name, int& w, int& h) {
    const std::string path = tempPath(name);
    {
        std::ofstream f(path, std::ios::binary);
        f << bytes;
    }
    ImageRef img;
    img.path = path;
    std::string why;
    std::vector<std::uint8_t> rgb;
    if (loadImagePixels(img, &why)) {
        w = img.pixelWidth, h = img.pixelHeight;
        for (std::size_t i = 0; i + 3 < img.pixels.size(); i += 4) rgb.insert(rgb.end(), {img.pixels[i], img.pixels[i + 1], img.pixels[i + 2]});
    }
    std::error_code ec;
    std::filesystem::remove(path, ec);
    return rgb;
}

double psnr(const std::vector<std::uint8_t>& a, const std::vector<std::uint8_t>& b) {
    if (a.size() != b.size() || a.empty()) return 0;
    double se = 0;
    for (std::size_t i = 0; i < a.size(); ++i) se += (double(a[i]) - double(b[i])) * (double(a[i]) - double(b[i]));
    return se == 0 ? 99 : 10 * std::log10(255.0 * 255.0 * double(a.size()) / se);
}

// A test picture: smooth gradients, and (withDisc) a disc of another colour on them.
std::vector<std::uint8_t> picture(int w, int h, int shift, bool withDisc = true) {
    std::vector<std::uint8_t> px;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const bool disc = withDisc && (x - w / 2 - shift) * (x - w / 2 - shift) + (y - h / 2) * (y - h / 2) < (h / 3) * (h / 3);
            px.push_back(static_cast<std::uint8_t>(disc ? 220 : 40 + 150 * x / w));
            px.push_back(static_cast<std::uint8_t>(disc ? 120 : 60 + 120 * y / h));
            px.push_back(static_cast<std::uint8_t>(disc ? 30 : 180));
        }
    return px;
}

std::uint32_t le32(const std::string& s, std::size_t at) {
    std::uint32_t v = 0;
    std::memcpy(&v, s.data() + at, 4);
    return v;
}
std::uint32_t be32(const std::string& s, std::size_t at) {
    return std::uint32_t(std::uint8_t(s[at])) << 24 | std::uint32_t(std::uint8_t(s[at + 1])) << 16 | std::uint32_t(std::uint8_t(s[at + 2])) << 8 |
           std::uint32_t(std::uint8_t(s[at + 3]));
}

} // namespace

TEST_CASE("JPEG: baseline files that decode to the picture, any size") {
    for (const auto& [w, h] : {std::pair{64, 48}, std::pair{37, 23}, std::pair{1, 1}, std::pair{17, 9}}) {
        const auto px = picture(w, h, 0, false);
        const std::string j = encodeJpeg(w, h, px, 90);
        REQUIRE(j.size() > 4);
        CHECK(std::uint8_t(j[0]) == 0xFF);
        CHECK(std::uint8_t(j[1]) == 0xD8);
        CHECK(std::uint8_t(j[j.size() - 2]) == 0xFF);
        CHECK(std::uint8_t(j[j.size() - 1]) == 0xD9);
        int dw = 0, dh = 0;
        const auto back = decode(j, "sicdm_test.jpg", dw, dh);
        CHECK(dw == w);
        CHECK(dh == h);
        if (w >= 17) CHECK(psnr(px, back) > 35);
    }
    // Sharp colour edges lose some to the colour being halved both ways (as every 4:2:0 JPEG).
    int ew = 0, eh = 0;
    CHECK(psnr(picture(64, 48, 0), decode(encodeJpeg(64, 48, picture(64, 48, 0), 90), "sicdm_edges.jpg", ew, eh)) > 24);
    // Noise: every Huffman symbol in use, big coefficients; still decodes, and quality counts.
    std::mt19937 rng(7);
    std::vector<std::uint8_t> noise(96 * 64 * 3);
    for (auto& v : noise) v = static_cast<std::uint8_t>(rng() & 0xFF);
    int dw = 0, dh = 0;
    const auto high = decode(encodeJpeg(96, 64, noise, 100), "sicdm_noise.jpg", dw, dh);
    REQUIRE(dw == 96);
    const auto low = decode(encodeJpeg(96, 64, noise, 20), "sicdm_noise2.jpg", dw, dh);
    CHECK(psnr(noise, high) > psnr(noise, low));
    CHECK(encodeJpeg(96, 64, noise, 100).size() > encodeJpeg(96, 64, noise, 20).size());
}

TEST_CASE("Video encoders: AVI, animated PNG and GIF are well formed") {
    const int w = 40, h = 30;
    std::vector<std::vector<std::uint8_t>> frames;
    for (int i = 0; i < 5; ++i) frames.push_back(picture(w, h, 3 * i));
    // AVI: RIFF, the header's counts, a JPEG chunk per frame and an index.
    std::vector<std::string> jpegs;
    for (const auto& f : frames) jpegs.push_back(encodeJpeg(w, h, f, 90));
    const std::string avi = encodeAviMjpeg(jpegs, w, h, 24);
    REQUIRE(avi.size() > 200);
    CHECK(avi.compare(0, 4, "RIFF") == 0);
    CHECK(le32(avi, 4) == avi.size() - 8);
    CHECK(avi.compare(8, 4, "AVI ") == 0);
    const std::size_t avih = avi.find("avih");
    REQUIRE(avih != std::string::npos);
    CHECK(le32(avi, avih + 8) == 1000000 / 24);
    CHECK(le32(avi, avih + 8 + 16) == 5); // frames
    CHECK(le32(avi, avih + 8 + 32) == 40);
    CHECK(avi.find("vidsMJPG") != std::string::npos);
    std::size_t chunks = 0;
    for (std::size_t at = avi.find("00dc"); at != std::string::npos; at = avi.find("00dc", at + 4))
        if (std::uint8_t(avi[at + 8]) == 0xFF && std::uint8_t(avi[at + 9]) == 0xD8) ++chunks;
    CHECK(chunks == 5);
    const std::size_t idx1 = avi.find("idx1");
    REQUIRE(idx1 != std::string::npos);
    CHECK(le32(avi, idx1 + 4) == 5 * 16);
    // APNG: its frames counted, sequence numbers in order, the first frame exact.
    const std::string png = encodeApng(frames, w, h, 24);
    REQUIRE(png.size() > 8);
    std::vector<std::string> types;
    std::vector<std::uint32_t> seq;
    std::uint32_t declared = 0;
    for (std::size_t at = 8; at + 12 <= png.size();) {
        const std::uint32_t len = be32(png, at);
        const std::string type = png.substr(at + 4, 4);
        types.push_back(type);
        if (type == "acTL") declared = be32(png, at + 8);
        if (type == "fcTL" || type == "fdAT") seq.push_back(be32(png, at + 8));
        at += 12 + len;
    }
    CHECK(declared == 5);
    CHECK(types.front() == "IHDR");
    CHECK(types[1] == "acTL");
    CHECK(types.back() == "IEND");
    CHECK(std::count(types.begin(), types.end(), "fcTL") == 5);
    for (std::size_t i = 0; i < seq.size(); ++i) CHECK(seq[i] == i);
    int dw = 0, dh = 0;
    CHECK(decode(png, "sicdm_test.png", dw, dh) == frames[0]);
    // GIF: a frame per picture, the first one decoding close to it.
    const std::string gif = encodeGif(frames, w, h, 25);
    CHECK(gif.compare(0, 6, "GIF89a") == 0);
    CHECK(gif.back() == '\x3B');
    CHECK(gif.find("NETSCAPE2.0") != std::string::npos);
    const auto first = decode(gif, "sicdm_test.gif", dw, dh);
    CHECK(dw == w);
    CHECK(psnr(frames[0], first) > 24);
}

TEST_CASE("GIF: the LZW stream survives a full code table, many times over") {
    // 64 colours far apart (no dithering crosses between them), at random: the codes fill the
    // table and start again many times, and the picture must come back exactly.
    const int w = 211, h = 157;
    std::mt19937 rng(3);
    std::vector<std::uint8_t> px;
    for (int i = 0; i < w * h; ++i) {
        const unsigned c = rng() % 64;
        px.insert(px.end(), {std::uint8_t((c & 3) * 85), std::uint8_t(((c >> 2) & 3) * 85), std::uint8_t(((c >> 4) & 3) * 85)});
    }
    int dw = 0, dh = 0;
    const auto back = decode(encodeGif({px}, w, h, 10), "sicdm_lzw.gif", dw, dh);
    REQUIRE(dw == w);
    CHECK(back == px);
    // And a flat picture (long runs: the longest strings).
    std::vector<std::uint8_t> flat(static_cast<std::size_t>(w * h * 3), 170);
    CHECK(decode(encodeGif({flat}, w, h, 10), "sicdm_flat.gif", dw, dh) == flat);
}

TEST_CASE("VIDEO: turntables and animations to each format") {
    Document d;
    REQUIRE(run(d, "BOX 40 20 6").ok);
    REQUIRE(run(d, "CYLINDER 5 20 @-15,0,6").ok);
    const std::string avi = tempPath("sicdm_video.avi"), png = tempPath("sicdm_video.png"), gif = tempPath("sicdm_video.gif");
    const std::string folder = tempPath("sicdm_video_frames") + "/";
    const std::string small = " SIZE 48 32 SAMPLES 1 FPS 4 SECONDS 1";
    auto r = run(d, "VIDEO " + avi + small);
    REQUIRE(r.ok);
    CHECK(r.lines[0].find("4 frames") != std::string::npos);
    const std::string a = readAll(avi);
    CHECK(a.compare(0, 4, "RIFF") == 0);
    CHECK(le32(a, a.find("avih") + 8 + 16) == 4);
    REQUIRE(run(d, "VIDEO " + png + small).ok);
    REQUIRE(run(d, "VIDEO " + gif + small).ok);
    REQUIRE(run(d, "VIDEO " + folder + small).ok);
    CHECK(std::filesystem::exists(folder + "frame_00004.png"));
    // A turntable's frames differ (the camera moves).
    int w = 0, h = 0;
    const auto f1 = decode(readAll(folder + "frame_00001.png"), "sicdm_f1.png", w, h);
    const auto f3 = decode(readAll(folder + "frame_00003.png"), "sicdm_f3.png", w, h);
    CHECK(w == 48);
    CHECK(psnr(f1, f3) < 40);
    std::error_code ec;
    std::filesystem::remove_all(folder, ec);
    std::filesystem::remove(avi, ec), std::filesystem::remove(png, ec), std::filesystem::remove(gif, ec);

    // An animation: the camera still, the block turning; frame 1 and frame 3 differ, a
    // turntable of nothing moving (TURNS 0) doesn't.
    REQUIRE(run(d, "MOTION CIRCLE Spin AXIS Z AT 0,0,0 SPEED 90 BODIES \"Block 1\"").ok);
    REQUIRE(run(d, "VIDEO " + folder + small).ok); // animation: there is a motion
    const auto a1 = decode(readAll(folder + "frame_00001.png"), "sicdm_a1.png", w, h);
    const auto a3 = decode(readAll(folder + "frame_00003.png"), "sicdm_a3.png", w, h);
    CHECK(psnr(a1, a3) < 40);
    std::filesystem::remove_all(folder, ec);
    REQUIRE(run(d, "VIDEO " + folder + small + " ANIMATION FROM 0 TO 1 TURNS 0").ok);
    REQUIRE(run(d, "MOTION OFF Spin").ok);
    std::filesystem::remove_all(folder, ec);
    REQUIRE(run(d, "VIDEO " + folder + small + " ANIMATION TURNS 0").ok);
    const auto s1 = decode(readAll(folder + "frame_00001.png"), "sicdm_s1.png", w, h);
    const auto s3 = decode(readAll(folder + "frame_00003.png"), "sicdm_s3.png", w, h);
    CHECK(s1 == s3); // nothing moves, the camera is still: the same picture
    std::filesystem::remove_all(folder, ec);
}

TEST_CASE("VIDEO: stopping early, refusals, and over MCP") {
    Document d;
    VideoRequest rq;
    rq.path = tempPath("sicdm_empty.avi");
    std::string why;
    CHECK(!makeVideo(d.scene, rq, nullptr, &why)); // nothing to film
    REQUIRE(run(d, "BOX 10 10 10").ok);
    rq.render.width = 32, rq.render.height = 24, rq.render.samples = 1;
    rq.fps = 10, rq.seconds = 1;
    VideoReport rep;
    REQUIRE(makeVideo(d.scene, rq, &rep, &why, [](int done, int) { return done < 3; }));
    CHECK(rep.stopped);
    CHECK(rep.frames == 3);
    CHECK(le32(readAll(rq.path), readAll(rq.path).find("avih") + 8 + 16) == 3);
    std::error_code ec;
    std::filesystem::remove(rq.path, ec);
    CHECK(!run(d, "VIDEO " + tempPath("x.mp4")).ok);
    CHECK(run(d, "VIDEO " + tempPath("x.mp4")).lines[0].find("ffmpeg") != std::string::npos);
    CHECK(!run(d, "VIDEO").ok);
    CHECK(!run(d, "VIDEO x.avi SECONDS 0").ok);
    CHECK(!run(d, "VIDEO x.avi FPS 500").ok);
    CHECK(!run(d, "VIDEO x.avi SIZE 8 8").ok);
    CHECK(!run(d, "VIDEO x.avi FROM 2 TO 1").ok);
    CHECK(!run(d, "VIDEO x.avi VIEW CURRENT").ok); // no app window
    CHECK(!run(d, "VIDEO x.avi VIEW SIDEWAYS").ok);
    CHECK(!run(d, "VIDEO x.avi WOBBLE").ok);
    CHECK(!std::filesystem::exists("x.avi"));

    McpServer server(d, nullptr);
    const std::string path = tempPath("sicdm_mcp_video.gif");
    Json args = Json::object();
    args.set("path", path).set("seconds", 0.5).set("fps", 4).set("samples", 1).set("size", Json(Json::Array{Json(32), Json(24)}));
    const std::string msg = R"({"jsonrpc":"2.0","id":4,"method":"tools/call","params":{"name":"make_video","arguments":)" + args.dump() + "}}";
    const auto r = Json::parse(server.handle(msg));
    REQUIRE(r.has_value());
    CHECK(!(*r)["result"]["isError"].asBool());
    CHECK(readAll(path).compare(0, 6, "GIF89a") == 0);
    std::filesystem::remove(path, ec);
}

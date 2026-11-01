#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/VectorImport.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

using namespace sicdm;
namespace fs = std::filesystem;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

// zlib data with one stored (uncompressed) deflate block: what FlateDecode reads.
std::string zlibStored(const std::string& data) {
    std::string z = "\x78\x01";
    z += '\x01'; // final block, stored
    const std::uint16_t len = static_cast<std::uint16_t>(data.size()), nlen = static_cast<std::uint16_t>(~len);
    z += static_cast<char>(len & 0xFF), z += static_cast<char>(len >> 8);
    z += static_cast<char>(nlen & 0xFF), z += static_cast<char>(nlen >> 8);
    z += data;
    std::uint32_t a = 1, b = 0;
    for (unsigned char c : data) a = (a + c) % 65521, b = (b + a) % 65521;
    const std::uint32_t adler = (b << 16) | a;
    for (int k = 3; k >= 0; --k) z += static_cast<char>((adler >> (8 * k)) & 0xFF);
    return z;
}

std::string streamObj(int n, const std::string& dictExtra, const std::string& data) {
    return std::to_string(n) + " 0 obj\n<< /Length " + std::to_string(data.size()) + " " + dictExtra + " >>\nstream\n" + data + "\nendstream\nendobj\n";
}

struct Box {
    float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
};
Box boxOf(const VectorPath& p) {
    Box b;
    for (const Vec2& q : p.pts) b.x0 = std::min(b.x0, q.x), b.y0 = std::min(b.y0, q.y), b.x1 = std::max(b.x1, q.x), b.y1 = std::max(b.y1, q.y);
    return b;
}
bool near(float a, float b, float tol = 0.01f) { return std::fabs(a - b) < tol; }

} // namespace

TEST_CASE("PDF import: strokes and fills through transforms, compressed streams, object streams and forms; clips and text left out") {
    const double pt = 72.0 / 25.4; // points per mm
    // Page 1's content (compressed): a 30 x 20 mm rectangle at (10, 10) mm, a stroked line,
    // a clipping path (not drawn), text (not drawn), and a form drawn moved by 50 mm.
    char buf[512];
    std::snprintf(buf, sizeof buf,
                  "q %.6f 0 0 %.6f 0 0 cm\n10 10 30 20 re S\n5 45 m 95 45 l S\nQ\n0 0 100 100 re W n\nBT /F1 12 Tf (hello) Tj ET\nq 1 0 0 1 %.6f 0 cm /Fm1 Do Q\n",
                  pt, pt, 50 * pt);
    const std::string content = buf;
    // The form: a quarter circle (one cubic) and a filled triangle, in mm through its matrix.
    char form[256];
    std::snprintf(form, sizeof form, "0 0 m 10 0 l 10 5.5228 5.5228 10 0 10 c S\n0 20 m 10 20 l 5 28 l f\n");
    std::string pdf = "%PDF-1.5\n";
    pdf += "1 0 obj\n<< /Type /Catalog /Pages 2 0 R >>\nendobj\n";
    pdf += "2 0 obj\n<< /Type /Pages /Kids [3 0 R 6 0 R] /Count 2 /MediaBox [0 0 " + std::to_string(100 * pt) + " " + std::to_string(50 * pt) + "] >>\nendobj\n";
    // Page 1 and its resources live in an object stream.
    const std::string obj3 = "<< /Type /Page /Parent 2 0 R /Contents 4 0 R /Resources << /XObject << /Fm1 5 0 R >> >> >>";
    const std::string header = "3 0 ";
    pdf += streamObj(7, "/Type /ObjStm /N 1 /First " + std::to_string(header.size()), header + obj3);
    pdf += streamObj(4, "/Filter /FlateDecode", zlibStored(content));
    char mat[128];
    std::snprintf(mat, sizeof mat, "/Type /XObject /Subtype /Form /BBox [0 0 100 100] /Matrix [%.6f 0 0 %.6f 0 0]", pt, pt);
    pdf += streamObj(5, mat, form);
    pdf += "6 0 obj\n<< /Type /Page /Parent 2 0 R /Contents 8 0 R >>\nendobj\n";
    pdf += streamObj(8, "", "0 0 m 72 72 l S\n");
    pdf += "trailer\n<< /Root 1 0 R >>\n%%EOF\n";

    VectorImportReport rep;
    std::string err;
    const std::vector<VectorPath> paths = parsePdf(pdf, 1, 1.0, &rep, &err);
    CHECK(err.empty());
    CHECK(rep.pages == 2 && near(rep.size.x, 100.0f) && near(rep.size.y, 50.0f));
    REQUIRE(paths.size() == 4); // rectangle, line, quarter circle, triangle; not the clip
    const Box r = boxOf(paths[0]);
    CHECK(paths[0].closed && paths[0].pts.size() == 4 && near(r.x0, 10) && near(r.y0, 10) && near(r.x1, 40) && near(r.y1, 30));
    CHECK(!paths[1].closed && paths[1].pts.size() == 2 && near(paths[1].pts[0].x, 5) && near(paths[1].pts[1].x, 95));
    // The arc: flattened, every point 10 mm from the form's origin (moved to x 50).
    REQUIRE(paths[2].pts.size() > 8);
    float worst = 0;
    for (std::size_t k = 1; k < paths[2].pts.size(); ++k) {
        const Vec2 q = paths[2].pts[k];
        if (q.x < 50.5f && q.y < 0.5f) continue; // the straight start
        worst = std::max(worst, std::fabs(std::hypot(q.x - 50.0f, q.y) - 10.0f));
    }
    CHECK(worst < 0.03f);
    CHECK(paths[3].closed && paths[3].pts.size() == 3 && near(boxOf(paths[3]).y1, 28)); // filled: closed
    // Page 2, at scale 1:10.
    const std::vector<VectorPath> p2 = parsePdf(pdf, 2, 10.0, &rep, &err);
    REQUIRE(p2.size() == 1);
    CHECK(near(p2[0].pts[1].x, 254.0f, 0.05f)); // 72 pt = 25.4 mm, x10
    CHECK(parsePdf(pdf, 3, 1.0, &rep, &err).empty() && err.find("2 pages") != std::string::npos);
    CHECK(parsePdf("not a pdf", 1, 1.0, &rep, &err).empty());
}

TEST_CASE("SVG import: paths with every command, shapes, transforms and the document's size") {
    const std::string svg = R"SVG(<?xml version="1.0"?>
<svg xmlns="http://www.w3.org/2000/svg" width="200mm" height="100mm" viewBox="0 0 400 200">
  <defs><path id="hidden" d="M0 0 L 400 200"/></defs>
  <rect x="20" y="20" width="60" height="40" fill="none" stroke="black"/>
  <circle cx="300" cy="100" r="40"/>
  <g transform="translate(100 100) scale(2)">
    <line x1="0" y1="0" x2="10" y2="0"/>
  </g>
  <path d="M 200 20 h 40 v 40 H 200 z m 0 60 a 20 20 0 0 1 40 0"/>
  <ellipse cx="50" cy="150" rx="30" ry="10"/>
</svg>)SVG";
    VectorImportReport rep;
    std::string err;
    const std::vector<VectorPath> paths = parseSvg(svg, 1.0, &rep, &err);
    CHECK(err.empty() && near(rep.size.x, 200) && near(rep.size.y, 100));
    REQUIRE(paths.size() == 6);
    // 1 user unit = 0.5 mm; y turned up (top of the drawing at y 100 mm).
    const Box r = boxOf(paths[0]);
    CHECK(paths[0].closed && near(r.x0, 10) && near(r.x1, 40) && near(r.y0, 70) && near(r.y1, 90));
    CHECK(paths[1].circle && near(paths[1].pts[0].x, 150) && near(paths[1].pts[0].y, 50) && near(paths[1].radius, 20));
    CHECK(!paths[2].closed && near(paths[2].pts[0].x, 50) && near(paths[2].pts[1].x, 60)); // translated and scaled
    CHECK(paths[3].closed && paths[3].pts.size() == 4 && near(boxOf(paths[3]).x1, 120));
    // The half circle (an arc of radius 20 user units = 10 mm about (220, 80)): points on it.
    float worst = 0;
    for (const Vec2& q : paths[4].pts) worst = std::max(worst, std::fabs(std::hypot(q.x - 110.0f, q.y - 60.0f) - 10.0f));
    CHECK(worst < 0.01f && near(boxOf(paths[4]).y1, 70.0f, 0.05f)); // it bulges up (y down in SVG)
    CHECK(paths[5].closed && near(boxOf(paths[5]).x1, 40) && near(boxOf(paths[5]).y1, 30)); // the ellipse
    CHECK(parseSvg("<html></html>", 1.0, &rep, &err).empty());
}

TEST_CASE("IMPORT of a PDF and an SVG puts their curves in the sketch, at a scale and page") {
    const fs::path dir = fs::temp_directory_path() / "sicdm_tests" / "vector";
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir);
    {
        std::ofstream f(dir / "a.svg");
        f << R"(<svg xmlns="http://www.w3.org/2000/svg" width="100mm" height="100mm" viewBox="0 0 100 100"><rect x="10" y="10" width="20" height="20"/><circle cx="50" cy="50" r="5"/></svg>)";
    }
    {
        std::ofstream f(dir / "b.pdf", std::ios::binary);
        f << "%PDF-1.4\n1 0 obj\n<< /Type /Catalog /Pages 2 0 R >>\nendobj\n2 0 obj\n<< /Type /Pages /Kids [3 0 R] /Count 1 >>\nendobj\n"
             "3 0 obj\n<< /Type /Page /Parent 2 0 R /MediaBox [0 0 200 200] /Contents 4 0 R >>\nendobj\n"
          << streamObj(4, "", "0 0 m 72 0 l S\n") << "trailer\n<< /Root 1 0 R >>\n%%EOF\n";
    }
    Document d;
    const CommandOutput a = executeCommand(d, "IMPORT \"" + (dir / "a.svg").string() + "\"", nullptr);
    CHECK(a.ok);
    REQUIRE(d.scene.sketch().curves().size() == 2);
    CHECK(d.scene.sketch().curves()[1].kind == CurveKind::Circle && near(d.scene.sketch().curves()[1].radius, 5));
    const CommandOutput b = executeCommand(d, "IMPORT \"" + (dir / "b.pdf").string() + "\" SCALE 1:2", nullptr);
    CHECK(b.ok && b.lines[0].find("page 1 of 1") != std::string::npos);
    REQUIRE(d.scene.sketch().curves().size() == 3);
    CHECK(d.scene.sketch().curves()[2].kind == CurveKind::Line && near(d.scene.sketch().curves()[2].pts[1].x, 50.8f, 0.01f));
    CHECK(!executeCommand(d, "IMPORT \"" + (dir / "b.pdf").string() + "\" PAGE 4", nullptr).ok);
}

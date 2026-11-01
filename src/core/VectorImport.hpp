#pragma once

// Vector drawings into the sketch: the line work of a PDF page or an SVG file becomes
// sketch curves (lines, polylines; curves flattened finely; SVG circles stay circles).
//
// PDF: the page's content streams (FlateDecode, ASCIIHex, ASCII85 or none), object
// streams, form XObjects and the transforms (cm, q/Q, form matrices) are read; paths that
// are stroked or filled are kept, clipping paths and text are not. One PDF point is 1/72 in;
// SCALE multiplies (a drawing at 1:50 comes in at full size with SCALE 50).
// SVG: path (every command, arcs included), line, polyline, polygon, rect, circle and
// ellipse elements, nested <g> transforms, and the document's size (mm, cm, in, pt, px at
// 96 per inch) from width/height and viewBox. SVG's y axis points down: it is turned up.

#include "core/Math.hpp"
#include "core/Scene.hpp"

#include <string>
#include <vector>

namespace sicdm {

struct VectorPath {
    std::vector<Vec2> pts; // mm
    bool closed = false;
    bool circle = false;   // pts[0] the centre, radius in `radius`
    float radius = 0.0f;
};

struct VectorImportReport {
    std::string format;           // "PDF 1.7", "SVG"
    int pages = 0, page = 1;      // PDF
    Vec2 size{};                  // the page or document, mm
    std::vector<std::string> warnings;
};

std::vector<VectorPath> readPdfPaths(const std::string& path, int page, double scale, VectorImportReport* report, std::string* error);
std::vector<VectorPath> readSvgPaths(const std::string& path, double scale, VectorImportReport* report, std::string* error);
// The same from text in memory (tests).
std::vector<VectorPath> parsePdf(const std::string& bytes, int page, double scale, VectorImportReport* report, std::string* error);
std::vector<VectorPath> parseSvg(const std::string& text, double scale, VectorImportReport* report, std::string* error);

// Adds them to the sketch on `plane`; the number of curves added.
int addVectorPaths(Scene& scene, SketchPlane plane, const PlaneFrame& frame, const std::vector<VectorPath>& paths);

} // namespace sicdm

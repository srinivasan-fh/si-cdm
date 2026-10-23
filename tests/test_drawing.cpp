#include "TestFramework.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Drawing.hpp"
#include "core/FileIO.hpp"
#include "core/Json.hpp"
#include "core/Scene.hpp"

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }
std::string tempPath(const char* name) { return (std::filesystem::temp_directory_path() / name).string(); }
std::string stepPath(const std::string& name) { return std::string(SICDM_SOURCE_DIR) + "/tests/data/step/" + name + ".step"; }

std::string readAll(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream s;
    s << f.rdbuf();
    return s.str();
}

// A 60 x 40 x 15 block with two through holes and a pocket in its top.
void makeBracket(Document& d) {
    run(d, "BOX 60 40 15 @0,0,0");
    run(d, "HOLE \"Block 1\" AT -20,0,15 DIAMETER 6 THROUGH");
    run(d, "HOLE \"Block 1\" AT 20,0,15 DIAMETER 6 THROUGH");
    run(d, "POCKET \"Block 1\" AT 0,0,15 BOX WIDTH 16 LENGTH 24 DEPTH 6");
}

// Each view's box on the sheet, from the points the dimension tool snaps to.
std::map<std::string, std::array<double, 4>> viewBoxes(const SheetGraphics& g) {
    std::map<std::string, std::array<double, 4>> boxes;
    for (const SheetPoint& p : g.snaps) {
        auto [it, fresh] = boxes.try_emplace(p.view, std::array<double, 4>{p.x, p.y, p.x, p.y});
        if (!fresh) {
            auto& b = it->second;
            b = {std::min(b[0], p.x), std::min(b[1], p.y), std::max(b[2], p.x), std::max(b[3], p.y)};
        }
    }
    return boxes;
}

bool overlap(const std::array<double, 4>& a, const std::array<double, 4>& b) {
    return a[0] < b[2] && a[2] > b[0] && a[1] < b[3] && a[3] > b[1];
}

std::vector<double> dimValues(const Drawing& d) {
    std::vector<double> v;
    for (const DrawingDim& dim : d.dims) v.push_back(dimensionValue(d, dim));
    return v;
}

bool hasValue(const std::vector<double>& v, double x) {
    for (double a : v)
        if (std::fabs(a - x) < 1e-3) return true;
    return false;
}

int count(const SheetGraphics& g, SheetLine::Style s) {
    int n = 0;
    for (const SheetLine& l : g.lines) n += l.style == s;
    return n;
}

bool hasText(const SheetGraphics& g, const std::string& t) {
    for (const SheetText& x : g.texts)
        if (x.text == t) return true;
    return false;
}

} // namespace

TEST_CASE("Drawing: standard views laid out apart, lined up, on the sheet and off the title block") {
    for (const bool third : {false, true}) {
        Document d;
        makeBracket(d);
        REQUIRE(run(d, third ? "SHEET NEW A3 THIRD" : "SHEET NEW A3").ok);
        const Drawing& dr = d.state.drawing;
        REQUIRE(dr.views.size() == 4);
        CHECK(dr.scale == 1.0); // 60 mm fits A3 at full size
        const DrawingView *front = dr.view("Front"), *top = dr.view("Top"), *right = dr.view("Right");
        REQUIRE(front && top && right && dr.view("Isometric"));
        // Projected views share the front view's axes.
        CHECK(std::fabs(top->x - front->x) < 1e-9);
        CHECK(std::fabs(right->y - front->y) < 1e-9);
        // First angle: the top view below the front and the right view to its left; third angle the other way.
        CHECK(third ? top->y > front->y : top->y < front->y);
        CHECK(third ? right->x > front->x : right->x < front->x);

        const SheetGraphics g = renderDrawing(dr, d.scene, 1);
        CHECK(g.warnings.empty());
        const auto boxes = viewBoxes(g);
        CHECK(boxes.size() == 4);
        const std::array<double, 4> block{g.width - 190, 10, g.width - 10, 50};
        for (auto i = boxes.begin(); i != boxes.end(); ++i) {
            const auto& b = i->second;
            CHECK(b[0] > 10 && b[1] > 10 && b[2] < g.width - 10 && b[3] < g.height - 10);
            CHECK(!overlap(b, block));
            for (auto j = std::next(i); j != boxes.end(); ++j) CHECK(!overlap(b, j->second));
        }
        // The front view is 60 x 15 at 1:1.
        const auto& fb = boxes.at("Front");
        CHECK(std::fabs((fb[2] - fb[0]) - 60) < 0.05);
        CHECK(std::fabs((fb[3] - fb[1]) - 15) < 0.05);
    }
}

TEST_CASE("Drawing: hidden lines dashed, and left out when the view asks") {
    Document d;
    makeBracket(d);
    REQUIRE(run(d, "SHEET NEW A3").ok);
    // The holes and the pocket are behind the front face.
    const SheetGraphics g = renderDrawing(d.state.drawing, d.scene, 1);
    CHECK(count(g, SheetLine::Visible) > 20);
    CHECK(count(g, SheetLine::Hidden) > 4);
    CHECK(count(g, SheetLine::Border) > 4);
    for (const char* view : {"Front", "Top", "Right"}) REQUIRE(run(d, std::string("SHEET VIEW HIDDEN ") + view + " OFF").ok);
    const SheetGraphics h = renderDrawing(d.state.drawing, d.scene, 1);
    CHECK(count(h, SheetLine::Hidden) == 0);
    CHECK(count(h, SheetLine::Visible) == count(g, SheetLine::Visible));
}

TEST_CASE("Drawing: a section view is hatched where it cuts, labelled, and the cut line drawn") {
    Document d;
    makeBracket(d);
    REQUIRE(run(d, "SHEET NEW A3").ok);
    const auto r = run(d, "SHEET SECTION");
    REQUIRE(r.ok);
    const Drawing& dr = d.state.drawing;
    REQUIRE(dr.views.size() == 5);
    const DrawingView& sec = dr.views.back();
    CHECK(sec.kind == ViewKind::Section);
    CHECK(sec.parent == "Front");
    CHECK(std::fabs(sec.cut) < 1e-9); // through the middle of the bracket
    const SheetGraphics g = renderDrawing(dr, d.scene, 1);
    CHECK(g.warnings.empty());
    CHECK(count(g, SheetLine::Thin) > 10); // hatching, dimension and title block lines
    CHECK(count(g, SheetLine::Cut) == 1);
    CHECK(hasText(g, "SECTION A-A"));
    CHECK(hasText(g, "A"));
    // The cut through x = 0 shows the bracket from the side: 40 wide, 15 high.
    const auto boxes = viewBoxes(g);
    REQUIRE(boxes.count(sec.name) == 1);
    const auto& b = boxes.at(sec.name);
    CHECK(std::fabs((b[2] - b[0]) - 40) < 0.05);
    CHECK(std::fabs((b[3] - b[1]) - 15) < 0.05);
    for (const auto& [name, o] : boxes)
        if (name != sec.name) CHECK(!overlap(b, o));
    // Without a parent it can be cut from, it is refused.
    CHECK(!run(d, "SHEET SECTION Isometric").ok);
}

TEST_CASE("Drawing: automatic dimensions measure the overall sizes and the hole diameters") {
    {
        Document d;
        makeBracket(d);
        REQUIRE(run(d, "SHEET NEW").ok);
        REQUIRE(run(d, "SHEET AUTODIM").ok);
        const auto v = dimValues(d.state.drawing);
        CHECK(hasValue(v, 60));
        CHECK(hasValue(v, 40));
        CHECK(hasValue(v, 15));
        // Run again: nothing new.
        const std::size_t n = d.state.drawing.dims.size();
        run(d, "SHEET AUTODIM");
        CHECK(d.state.drawing.dims.size() == n);
    }
    {
        Document d;
        REQUIRE(run(d, "IMPORT \"" + stepPath("plate_holes") + "\"").ok);
        REQUIRE(run(d, "SHEET NEW A4").ok);
        REQUIRE(run(d, "SHEET AUTODIM").ok);
        const Drawing& dr = d.state.drawing;
        const auto v = dimValues(dr);
        CHECK(hasValue(v, 60));
        CHECK(hasValue(v, 40));
        CHECK(hasValue(v, 10));
        int diameters = 0;
        for (const DrawingDim& dim : dr.dims) diameters += dim.kind == DrawingDim::Diameter;
        CHECK(diameters == 2);
        CHECK(hasValue(v, 8));
        CHECK(hasValue(v, 14));
        const SheetGraphics g = renderDrawing(dr, d.scene, 1);
        CHECK(hasText(g, "\xC3\x98" "8"));   // Ø8
        CHECK(hasText(g, "\xC3\x98" "14"));
        CHECK(count(g, SheetLine::Center) > 4); // centre marks on the holes
    }
}

TEST_CASE("Drawing: dimensions by hand, measured as the view shows them, and deleted") {
    Document d;
    makeBracket(d);
    REQUIRE(run(d, "SHEET NEW").ok);
    auto r = run(d, "SHEET DIM HORIZONTAL Front -20,0,15 20,0,15");
    REQUIRE(r.ok);
    REQUIRE(d.state.drawing.dims.size() == 1);
    CHECK(std::fabs(dimensionValue(d.state.drawing, d.state.drawing.dims[0]) - 40) < 1e-9);
    REQUIRE(run(d, "SHEET DIM VERTICAL Front -30,0,0 -30,0,15").ok);
    // Aligned: the length as the view shows it (the depth along the view does not count).
    REQUIRE(run(d, "SHEET DIM ALIGNED Front -30,-20,0 30,20,15").ok);
    CHECK(std::fabs(dimensionValue(d.state.drawing, d.state.drawing.dims[2]) - std::hypot(60.0, 15.0)) < 1e-9);
    CHECK(!run(d, "SHEET DIM HORIZONTAL Nowhere 0,0,0 1,0,0").ok);
    REQUIRE(run(d, "SHEET DIM DELETE 1").ok);
    CHECK(d.state.drawing.dims.size() == 2);
    REQUIRE(run(d, "SHEET DIM CLEAR").ok);
    CHECK(d.state.drawing.dims.empty());
}

TEST_CASE("Drawing: scale, sheet size and projection lay the views out again, and warn when they do not fit") {
    Document d;
    makeBracket(d);
    REQUIRE(run(d, "SHEET NEW A4").ok);
    REQUIRE(run(d, "SHEET SCALE 1:2").ok);
    CHECK(d.state.drawing.scale == 0.5);
    CHECK(renderDrawing(d.state.drawing, d.scene, 1).warnings.empty());
    // 5:1 puts a 300 mm wide front view on a 297 mm sheet.
    const auto r = run(d, "SHEET SCALE 5:1");
    REQUIRE(r.ok);
    bool warned = false;
    for (const std::string& l : r.lines) warned = warned || l.find("warning") != std::string::npos;
    CHECK(warned);
    CHECK(!renderDrawing(d.state.drawing, d.scene, 1).warnings.empty());
    REQUIRE(run(d, "SHEET SIZE A2").ok);
    CHECK(d.state.drawing.sheet == "A2");
    REQUIRE(run(d, "SHEET PROJECTION THIRD").ok);
    CHECK(d.state.drawing.thirdAngle);
    CHECK(!run(d, "SHEET SCALE nonsense").ok);
    CHECK(!run(d, "SHEET SIZE B7").ok);

    double s = 0;
    CHECK(parseScale("1:2", s) && s == 0.5);
    CHECK(parseScale("2:1", s) && s == 2);
    CHECK(scaleText(0.5) == "1:2");
    CHECK(scaleText(2) == "2:1");
    CHECK(standardScale(0.7) == 0.5);
    CHECK(standardScale(1.0) == 1.0);
}

TEST_CASE("Drawing: PDF, SVG and DXF exports are well formed") {
    Document d;
    makeBracket(d);
    REQUIRE(run(d, "SHEET NEW").ok);
    run(d, "SHEET SECTION");
    run(d, "SHEET AUTODIM");
    run(d, "SHEET TITLE TITLE \"Bracket <A&B>\"");
    run(d, "SHEET NOTE \"BREAK SHARP EDGES\" AT 30,40");

    const std::string pdf = tempPath("sicdm_drawing.pdf"), svg = tempPath("sicdm_drawing.svg"), dxf = tempPath("sicdm_drawing.dxf");
    REQUIRE(run(d, "SHEET EXPORT \"" + pdf + "\"").ok);
    REQUIRE(run(d, "SHEET EXPORT \"" + svg + "\"").ok);
    REQUIRE(run(d, "SHEET EXPORT \"" + dxf + "\"").ok);
    CHECK(!run(d, "SHEET EXPORT \"" + tempPath("sicdm_drawing.png") + "\"").ok);

    const std::string p = readAll(pdf);
    CHECK(p.rfind("%PDF-1.", 0) == 0);
    CHECK(p.find("%%EOF") != std::string::npos);
    CHECK(p.find("/MediaBox [0 0 1190.55 841.89]") != std::string::npos ||
          p.find("/MediaBox [0 0 1190.6 841.9]") != std::string::npos || p.find("/MediaBox") != std::string::npos);
    // The cross-reference offsets point at the objects.
    const std::size_t xref = p.rfind("startxref");
    REQUIRE(xref != std::string::npos);
    const long at = std::stol(p.substr(xref + 10));
    CHECK(p.compare(static_cast<std::size_t>(at), 4, "xref") == 0);
    CHECK(p.find("(BREAK SHARP EDGES)") != std::string::npos);

    const std::string s = readAll(svg);
    CHECK(s.find("<svg") != std::string::npos);
    CHECK(s.find("</svg>") != std::string::npos);
    CHECK(s.find("Bracket &lt;A&amp;B&gt;") != std::string::npos);
    CHECK(s.find("stroke-dasharray") != std::string::npos);

    const std::string x = readAll(dxf);
    CHECK(x.find("SECTION") != std::string::npos);
    CHECK(x.find("HIDDEN") != std::string::npos);
    CHECK(x.find("CENTER") != std::string::npos);
    CHECK(x.find("LINE") != std::string::npos);
    CHECK(x.find("EOF") != std::string::npos);
    CHECK(x.substr(x.size() - 5).find("EOF") != std::string::npos);
    for (const std::string& f : {pdf, svg, dxf}) std::filesystem::remove(f);
}

TEST_CASE("Drawing: kept in the project file and read back the same") {
    Document d;
    makeBracket(d);
    REQUIRE(run(d, "SHEET NEW A2 PORTRAIT THIRD").ok);
    run(d, "SHEET SECTION");
    run(d, "SHEET AUTODIM");
    run(d, "SHEET TITLE MATERIAL 6061-T6");
    run(d, "SHEET NOTE \"BREAK SHARP EDGES\" AT 30,40 HEIGHT 5");
    run(d, "SHEET VIEW SCALE Isometric 1:2");
    const std::string before = drawingToJson(d.state.drawing).dump();

    const std::string path = tempPath("sicdm_drawing.sicdm");
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene scene;
    ProjectState state;
    REQUIRE(loadProject(scene, state, path, &err));
    CHECK(drawingToJson(state.drawing).dump() == before);
    CHECK(state.drawing.sheet == "A2");
    CHECK(!state.drawing.landscape);
    CHECK(state.drawing.thirdAngle);
    CHECK(state.drawing.views.size() == 5);
    CHECK(state.drawing.notes.size() == 1);
    CHECK(state.drawing.title["MATERIAL"] == "6061-T6");
    std::filesystem::remove(path);

    // A project without a drawing stays without one.
    Document e;
    run(e, "BOX 10 10 10");
    REQUIRE(saveProject(e.scene, e.state, path, &err));
    ProjectState s2;
    Scene sc2;
    REQUIRE(loadProject(sc2, s2, path, &err));
    CHECK(s2.drawing.empty());
    std::filesystem::remove(path);
}

TEST_CASE("Drawing: commands need a drawing and bodies, and views come and go") {
    Document d;
    CHECK(!run(d, "SHEET VIEW ADD Bottom").ok);
    CHECK(!run(d, "SHEET NEW").ok); // nothing to draw
    makeBracket(d);
    REQUIRE(run(d, "SHEET NEW").ok);
    REQUIRE(run(d, "SHEET VIEW ADD Bottom").ok);
    CHECK(d.state.drawing.view("Bottom") != nullptr);
    REQUIRE(run(d, "SHEET VIEW ADD Bottom").ok); // a second one, named apart
    CHECK(d.state.drawing.view("Bottom 2") != nullptr);
    REQUIRE(run(d, "SHEET VIEW DELETE \"Bottom 2\"").ok);
    REQUIRE(run(d, "SHEET VIEW MOVE Bottom 100,100").ok);
    CHECK(d.state.drawing.view("Bottom")->x == 100);
    REQUIRE(run(d, "SHEET VIEW DELETE Bottom").ok);
    CHECK(d.state.drawing.view("Bottom") == nullptr);
    REQUIRE(run(d, "SHEET DELETE").ok);
    CHECK(d.state.drawing.empty());
}

// SHEET: the drawing sheet.

#include "core/DrawingCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Drawing.hpp"
#include "core/FileIO.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>

namespace sicdm {

namespace {

std::string lowerOf(const std::string& s) {
    std::string o;
    for (char c : s) o += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return o;
}

std::string mm(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.1f", v);
    return b;
}

std::string describeView(const Drawing& d, const DrawingView& v) {
    std::string s = v.name + " (" + viewKindName(v.kind) + ") at " + mm(v.x) + ", " + mm(v.y) + " mm";
    if (v.scale > 0) s += ", scale " + scaleText(v.scale);
    if (v.kind == ViewKind::Section) s += ", cut through " + v.parent + (v.vertical ? " vertically at " : " horizontally at ") + mm(v.cut);
    if (!v.hidden) s += ", no hidden lines";
    (void)d;
    return s;
}

const char* dimKindName(DrawingDim::Kind k) {
    switch (k) {
    case DrawingDim::Horizontal: return "horizontal";
    case DrawingDim::Vertical: return "vertical";
    case DrawingDim::Aligned: return "aligned";
    case DrawingDim::Diameter: return "diameter";
    case DrawingDim::Radius: return "radius";
    }
    return "";
}

// Lays the standard views out again (after the scale or sheet changes); other views
// find a free place.
void relayout(Drawing& d, const Scene& scene) {
    const Drawing fresh = defaultDrawing(scene, d.bodies, d.sheet, d.landscape, d.thirdAngle, d.scale);
    for (DrawingView& v : d.views)
        for (const DrawingView& f : fresh.views)
            if (v.kind == f.kind && v.kind != ViewKind::Section) v.x = f.x, v.y = f.y;
    for (DrawingView& v : d.views)
        if (v.kind != ViewKind::Section && !fresh.view(viewKindName(v.kind)) && v.kind != ViewKind::Front) placeView(d, scene, v);
    for (DrawingView& v : d.views)
        if (v.kind == ViewKind::Section) placeView(d, scene, v);
}

} // namespace

CommandOutput runSheetCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted, ViewHost* view) {
    CommandOutput out;
    Drawing& d = doc.state.drawing;
    const std::string sub = a.empty() ? "list" : lowerOf(a[0]);
    auto quotedArg = [&](std::size_t k) { return k < quoted.size() && quoted[k]; };
    auto changed = [&] { doc.dirty = true; };
    auto need = [&]() {
        if (d.empty()) {
            out.fail("No drawing yet: SHEET NEW makes one of the visible bodies");
            return false;
        }
        return true;
    };
    // What does not fit on the sheet after views move or the scale changes.
    auto warnFit = [&]() {
        if (d.empty()) return;
        for (const std::string& w : renderDrawing(d, doc.scene, 1).warnings) out.say("  warning: " + w);
    };
    auto rest = [&](std::size_t from) {
        std::string s;
        for (std::size_t k = from; k < a.size(); ++k) s += (s.empty() ? "" : " ") + a[k];
        return s;
    };

    if (sub == "list" || sub == "status" || sub == "info") {
        if (d.empty()) {
            out.say("No drawing. SHEET NEW [bodies] [SIZE A3] [PORTRAIT] [THIRD] [SCALE 1:2] makes one");
            return out;
        }
        out.say("Drawing: " + d.sheet + (d.landscape ? " landscape" : " portrait") + ", " + (d.thirdAngle ? "third" : "first") + "-angle projection, scale " +
                scaleText(d.scale) + ", " + (d.bodies.empty() ? std::string("all visible bodies") : std::to_string(d.bodies.size()) + " bodies"));
        for (const DrawingView& v : d.views) out.say("  view " + describeView(d, v));
        for (std::size_t i = 0; i < d.dims.size(); ++i)
            out.say("  dim " + std::to_string(i + 1) + ": " + dimKindName(d.dims[i].kind) + " in " + d.dims[i].view + " = " + mm(dimensionValue(d, d.dims[i])) + " mm");
        for (const auto& [k, v] : d.title) out.say("  title " + k + ": " + v);
        return out;
    }

    if (sub == "new") {
        // SHEET NEW [names...] [SIZE s] [PORTRAIT|LANDSCAPE] [FIRST|THIRD] [SCALE 1:n]
        std::vector<std::string> bodies;
        std::string sheet = "A3";
        bool landscape = true, third = false;
        double scale = 0;
        for (std::size_t i = 1; i < a.size(); ++i) {
            const std::string w = lowerOf(a[i]);
            if (!quotedArg(i) && (w == "size" || w == "sheet") && i + 1 < a.size()) {
                sheet = a[++i];
                double W, H;
                if (!sheetSize(sheet, W, H)) return out.fail("Sheet sizes: A4, A3, A2, A1, A0, LETTER, TABLOID, ANSIC, ANSID");
            } else if (!quotedArg(i) && (w == "portrait" || w == "landscape")) landscape = w == "landscape";
            else if (!quotedArg(i) && (w == "first" || w == "third" || w == "iso" || w == "ansi" || w == "asme")) third = w == "third" || w == "ansi" || w == "asme";
            else if (!quotedArg(i) && w == "scale" && i + 1 < a.size()) {
                if (!parseScale(a[++i], scale)) return out.fail("SCALE takes 1:2, 2:1 or a number");
            } else {
                double W, H;
                if (!quotedArg(i) && sheetSize(a[i], W, H)) {
                    sheet = a[i];
                    continue;
                }
                bool found = false;
                for (const Body& b : doc.scene.bodies())
                    if (lowerOf(b.name) == lowerOf(a[i])) bodies.push_back(b.name), found = true;
                if (!found) return out.fail("No body named " + a[i]);
            }
        }
        Drawing nd = defaultDrawing(doc.scene, bodies, toUpper(sheet), landscape, third, scale);
        if (nd.empty()) return out.fail("Nothing to draw: no visible bodies");
        nd.title = d.title.empty() ? nd.title : d.title;
        if (nd.title["TITLE"].empty()) nd.title["TITLE"] = !doc.path.empty() ? std::filesystem::path(doc.path).stem().string() : doc.name;
        d = nd;
        changed();
        if (view) view->showSheet(true);
        out.say("New " + d.sheet + " drawing at " + scaleText(d.scale) + ": front, top, right and isometric views (" + (d.thirdAngle ? "third" : "first") +
                " angle). SHEET AUTODIM adds dimensions, SHEET EXPORT file.pdf writes it");
        warnFit();
        return out;
    }

    if (sub == "delete" || sub == "clear") {
        d = Drawing{};
        changed();
        if (view) view->showSheet(false);
        out.say("Drawing deleted");
        return out;
    }

    if (sub == "show" || sub == "open" || sub == "hide" || sub == "close") {
        const bool on = sub == "show" || sub == "open";
        if (on && !need()) return out;
        if (!view || !view->showSheet(on)) return out.fail("The drawing window needs the app");
        out.say(on ? "Drawing shown" : "Drawing hidden");
        return out;
    }

    if (!need()) return out;

    if (sub == "view") {
        // SHEET VIEW ADD kind [AT x,y] [SCALE s] | MOVE name x,y | DELETE name | SCALE name s | HIDDEN name ON|OFF
        if (a.size() < 3) return out.fail("SHEET VIEW ADD FRONT|BACK|TOP|BOTTOM|LEFT|RIGHT|ISO [AT x,y] [SCALE 1:2] | MOVE name x,y | DELETE name");
        const std::string op = lowerOf(a[1]);
        if (op == "add") {
            ViewKind k;
            if (!viewKindFromName(a[2], &k) || k == ViewKind::Section) return out.fail("Views: FRONT, BACK, TOP, BOTTOM, LEFT, RIGHT, ISO (SHEET SECTION cuts one)");
            DrawingView v{viewKindName(k), k, 0, 0, 0, k != ViewKind::Iso, "", true, 0, false};
            for (int n = 2; d.view(v.name); ++n) v.name = std::string(viewKindName(k)) + " " + std::to_string(n);
            bool placed = false;
            for (std::size_t i = 3; i < a.size(); ++i) {
                const std::string w = lowerOf(a[i]);
                if (w == "at" && i + 1 < a.size()) {
                    const auto p = parseVec3(a[++i]);
                    if (!p) return out.fail("AT takes x,y on the sheet (mm)");
                    v.x = p->x, v.y = p->y, placed = true;
                } else if (w == "scale" && i + 1 < a.size()) {
                    if (!parseScale(a[++i], v.scale)) return out.fail("SCALE takes 1:2, 2:1 or a number");
                } else if (w == "nohidden") v.hidden = false;
                else if (w == "hidden") v.hidden = true;
            }
            d.views.push_back(v);
            if (!placed) placeView(d, doc.scene, d.views.back());
            changed();
            out.say("Added " + describeView(d, d.views.back()));
            warnFit();
            return out;
        }
        auto findView = [&](const std::string& n) -> DrawingView* {
            for (DrawingView& v : d.views)
                if (lowerOf(v.name) == lowerOf(n)) return &v;
            return nullptr;
        };
        DrawingView* v = findView(a[2]);
        if (!v) return out.fail("No view named " + a[2]);
        if (op == "move" && a.size() >= 4) {
            const auto p = parseVec3(a[3]);
            if (!p) return out.fail("MOVE takes x,y on the sheet (mm)");
            v->x = p->x, v->y = p->y;
        } else if (op == "delete" || op == "remove") {
            const std::string name = v->name;
            d.views.erase(d.views.begin() + (v - d.views.data()));
            d.dims.erase(std::remove_if(d.dims.begin(), d.dims.end(), [&](const DrawingDim& m) { return lowerOf(m.view) == lowerOf(name); }), d.dims.end());
            changed();
            out.say("Deleted view " + name + " and its dimensions");
            return out;
        } else if (op == "scale" && a.size() >= 4) {
            if (!parseScale(a[3], v->scale)) return out.fail("SCALE takes 1:2, 2:1 or a number");
        } else if (op == "hidden" && a.size() >= 4) {
            v->hidden = lowerOf(a[3]) != "off" && lowerOf(a[3]) != "no";
        } else return out.fail("SHEET VIEW MOVE name x,y | DELETE name | SCALE name s | HIDDEN name ON|OFF");
        changed();
        out.say("View " + describeView(d, *v));
        warnFit();
        return out;
    }

    if (sub == "section") {
        // SHEET SECTION [parent] [VERTICAL|HORIZONTAL] [AT value] [FLIP] [LABEL B]
        DrawingView v{"", ViewKind::Section, 0, 0, 0, false, "Front", true, 0, false};
        bool cutGiven = false;
        std::string label;
        for (std::size_t i = 1; i < a.size(); ++i) {
            const std::string w = lowerOf(a[i]);
            if (w == "vertical" || w == "horizontal") v.vertical = w == "vertical";
            else if (w == "flip") v.flip = true;
            else if (w == "at" && i + 1 < a.size()) {
                const auto c = parseLength(a[++i]);
                if (!c) return out.fail("AT takes a position along the view (model mm)");
                v.cut = *c, cutGiven = true;
            } else if (w == "label" && i + 1 < a.size()) label = toUpper(a[++i]).substr(0, 1);
            else if (d.view(a[i])) v.parent = d.view(a[i])->name;
            else return out.fail("No view named " + a[i]);
        }
        const DrawingView* p = d.view(v.parent);
        if (!p || p->kind == ViewKind::Section || p->kind == ViewKind::Iso) return out.fail("Cut a front, top or side view");
        if (!cutGiven) { // through the middle of the bodies
            DVec3 r, u, into;
            viewAxes(p->kind, r, u, into);
            const DVec3 axis = v.vertical ? r : u;
            double lo = 1e300, hi = -1e300;
            for (const Body& b : doc.scene.bodies()) {
                if (d.bodies.empty() ? !b.visible : std::find(d.bodies.begin(), d.bodies.end(), b.name) == d.bodies.end()) continue;
                const AABB bb = b.mesh.bounds();
                if (!bb.valid()) continue;
                for (int c = 0; c < 8; ++c) {
                    const DVec3 q{(c & 1 ? bb.max.x : bb.min.x) + b.offset.x, (c & 2 ? bb.max.y : bb.min.y) + b.offset.y, (c & 4 ? bb.max.z : bb.min.z) + b.offset.z};
                    lo = std::min(lo, ddot(q, axis)), hi = std::max(hi, ddot(q, axis));
                }
            }
            v.cut = lo <= hi ? 0.5 * (lo + hi) : 0;
        }
        if (label.empty()) {
            char c = 'A';
            for (const DrawingView& o : d.views)
                if (o.kind == ViewKind::Section) c = static_cast<char>(std::max<int>(c, o.name.empty() ? 'A' : o.name.back() + 1));
            label = std::string(1, c);
        }
        v.name = "Section " + label;
        if (d.view(v.name)) return out.fail("There is already a " + v.name);
        d.views.push_back(v);
        placeView(d, doc.scene, d.views.back());
        changed();
        out.say("Added " + describeView(d, d.views.back()));
        warnFit();
        return out;
    }

    if (sub == "dim" || sub == "dimension") {
        // SHEET DIM HORIZONTAL|VERTICAL|ALIGNED view x,y,z x,y,z [OFFSET mm] [TEXT t]
        // SHEET DIM DIAMETER|RADIUS view centre point | DELETE n | CLEAR
        if (a.size() >= 2 && lowerOf(a[1]) == "clear") {
            d.dims.clear();
            changed();
            out.say("All dimensions removed");
            return out;
        }
        if (a.size() >= 3 && (lowerOf(a[1]) == "delete" || lowerOf(a[1]) == "remove")) {
            const auto n = parseNumber(a[2]);
            if (!n || *n < 1 || *n > static_cast<float>(d.dims.size())) return out.fail("No dimension " + a[2] + " (SHEET lists them)");
            d.dims.erase(d.dims.begin() + static_cast<std::ptrdiff_t>(*n) - 1);
            changed();
            out.say("Dimension removed");
            return out;
        }
        if (a.size() < 5) return out.fail("SHEET DIM HORIZONTAL|VERTICAL|ALIGNED|DIAMETER|RADIUS view x,y,z x,y,z [OFFSET mm] [TEXT t]");
        DrawingDim dm;
        const std::string k = lowerOf(a[1]);
        if (k == "horizontal" || k == "h") dm.kind = DrawingDim::Horizontal;
        else if (k == "vertical" || k == "v") dm.kind = DrawingDim::Vertical;
        else if (k == "aligned" || k == "a") dm.kind = DrawingDim::Aligned;
        else if (k == "diameter" || k == "dia") dm.kind = DrawingDim::Diameter;
        else if (k == "radius" || k == "r") dm.kind = DrawingDim::Radius;
        else return out.fail("Dimension kinds: HORIZONTAL, VERTICAL, ALIGNED, DIAMETER, RADIUS");
        const DrawingView* v = d.view(a[2]);
        if (!v) return out.fail("No view named " + a[2]);
        dm.view = v->name;
        const auto p0 = parseVec3(a[3]), p1 = parseVec3(a[4]);
        if (!p0 || !p1) return out.fail("Points are model x,y,z (mm)");
        dm.a = DVec3(*p0), dm.b = DVec3(*p1);
        dm.offset = dm.kind == DrawingDim::Diameter || dm.kind == DrawingDim::Radius ? 6 : 10;
        for (std::size_t i = 5; i < a.size(); ++i) {
            const std::string w = lowerOf(a[i]);
            if (w == "offset" && i + 1 < a.size()) {
                const auto o = parseNumber(a[++i]);
                if (!o) return out.fail("OFFSET takes sheet mm (negative: below / left)");
                dm.offset = *o;
            } else if (w == "text" && i + 1 < a.size()) dm.text = rest(++i), i = a.size();
        }
        d.dims.push_back(dm);
        changed();
        out.say("Dimension " + std::to_string(d.dims.size()) + ": " + dimKindName(dm.kind) + " " + mm(dimensionValue(d, dm)) + " mm in " + dm.view);
        return out;
    }

    if (sub == "autodim" || sub == "autodimension") {
        const int n = autoDimension(d, doc.scene, a.size() > 1 ? a[1] : "");
        if (n) changed();
        out.say(n ? "Added " + std::to_string(n) + " dimensions: overall sizes and hole diameters" : "Nothing new to dimension");
        return out;
    }

    if (sub == "title") {
        // SHEET TITLE field text   (TITLE, PART, MATERIAL, DRAWN, CHECKED, DATE, REV, SHEET, COMPANY)
        if (a.size() < 2) return out.fail("SHEET TITLE TITLE|PART|MATERIAL|DRAWN|CHECKED|DATE|REV|SHEET|COMPANY text");
        static const char* kFields[] = {"TITLE", "PART", "MATERIAL", "DRAWN", "CHECKED", "DATE", "REV", "SHEET", "COMPANY"};
        const std::string f = toUpper(a[1]);
        if (std::find_if(std::begin(kFields), std::end(kFields), [&](const char* x) { return f == x; }) == std::end(kFields))
            return out.fail("Title fields: TITLE, PART, MATERIAL, DRAWN, CHECKED, DATE, REV, SHEET, COMPANY");
        d.title[f] = rest(2);
        changed();
        out.say("Title block " + f + ": " + d.title[f]);
        return out;
    }

    if (sub == "note") {
        // SHEET NOTE text [AT x,y] [HEIGHT h]
        DrawingNote n{20, 60, 3.5, ""};
        std::string text;
        for (std::size_t i = 1; i < a.size(); ++i) {
            const std::string w = lowerOf(a[i]);
            if (!quotedArg(i) && w == "at" && i + 1 < a.size()) {
                const auto p = parseVec3(a[++i]);
                if (!p) return out.fail("AT takes x,y on the sheet (mm)");
                n.x = p->x, n.y = p->y;
            } else if (!quotedArg(i) && w == "height" && i + 1 < a.size()) {
                const auto h = parseNumber(a[++i]);
                if (!h || !(*h > 0)) return out.fail("HEIGHT takes mm");
                n.height = *h;
            } else text += (text.empty() ? "" : " ") + a[i];
        }
        if (text.empty()) return out.fail("SHEET NOTE text [AT x,y] [HEIGHT mm]");
        n.text = text;
        d.notes.push_back(n);
        changed();
        out.say("Note added");
        return out;
    }

    if (sub == "scale" || sub == "size" || sub == "projection") {
        if (a.size() < 2) return out.fail("SHEET SCALE 1:2 | SHEET SIZE A2 [PORTRAIT] | SHEET PROJECTION FIRST|THIRD");
        if (sub == "scale") {
            double s = 0;
            if (!parseScale(a[1], s)) return out.fail("SCALE takes 1:2, 2:1 or a number");
            d.scale = s;
        } else if (sub == "size") {
            double W, H;
            if (!sheetSize(a[1], W, H)) return out.fail("Sheet sizes: A4, A3, A2, A1, A0, LETTER, TABLOID, ANSIC, ANSID");
            d.sheet = toUpper(a[1]);
            if (a.size() > 2) d.landscape = lowerOf(a[2]) != "portrait";
        } else {
            d.thirdAngle = lowerOf(a[1]) == "third" || lowerOf(a[1]) == "ansi" || lowerOf(a[1]) == "asme";
        }
        relayout(d, doc.scene);
        changed();
        out.say("Drawing: " + d.sheet + ", scale " + scaleText(d.scale) + ", " + (d.thirdAngle ? "third" : "first") + "-angle; views laid out again");
        warnFit();
        return out;
    }

    if (sub == "export" || sub == "save") {
        if (a.size() < 2) return out.fail("SHEET EXPORT file.pdf|.svg|.dxf");
        const std::string path = rest(1);
        std::string ext = lowerOf(std::filesystem::path(path).extension().string());
        const SheetGraphics g = renderDrawing(d, doc.scene, 2);
        std::string data;
        if (ext == ".pdf") data = sheetToPdf(g, d.title.count("TITLE") ? d.title.at("TITLE") : "Drawing");
        else if (ext == ".svg") data = sheetToSvg(g);
        else if (ext == ".dxf") data = sheetToDxf(g);
        else return out.fail("SHEET EXPORT writes .pdf, .svg or .dxf");
        std::string err;
        if (!writeFile(path, data, &err)) return out.fail(err);
        out.say("Wrote the " + d.sheet + " drawing to " + path + " (" + std::to_string(d.views.size()) + " views, " + std::to_string(d.dims.size()) + " dimensions)");
        for (const std::string& w : g.warnings) out.say("  warning: " + w);
        return out;
    }

    return out.fail("SHEET NEW | VIEW | SECTION | DIM | AUTODIM | TITLE | NOTE | SCALE | SIZE | PROJECTION | EXPORT | SHOW | HIDE | DELETE");
}

} // namespace sicdm

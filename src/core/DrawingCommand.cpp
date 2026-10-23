// SHEET: the drawing sheet.

#include "core/DrawingCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Drawing.hpp"
#include "core/FileIO.hpp"
#include "core/Tolerances.hpp"
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

std::string mm3(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.6g", v);
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

// Reads a size tolerance from words[i...]: a number (+/-), "+a -b" or "+a/-b"
// (deviations), LIMITS +a -b, FIT H7 [VALUES], BASIC or NONE. Advances i past it.
bool parseTolerance(const std::vector<std::string>& w, std::size_t& i, DimTolerance& t, std::string& why) {
    auto number = [&](const std::string& s, double& v) {
        std::string x = s;
        if (x.rfind("\xC2\xB1", 0) == 0) x = x.substr(2);
        if (!x.empty() && x[0] == '+') x = x.substr(1);
        const auto n = parseNumber(x);
        if (!n) return false;
        v = *n;
        return true;
    };
    if (i >= w.size()) return why = "TOL takes 0.1 (+/-), +0.1 -0.05, LIMITS +0.1 -0.05, FIT H7 [VALUES], BASIC or NONE", false;
    const std::string k = lowerOf(w[i]);
    if (k == "none" || k == "off") {
        t = DimTolerance{};
        ++i;
        return true;
    }
    if (k == "basic") {
        t = DimTolerance{};
        t.kind = DimTolerance::Basic;
        ++i;
        return true;
    }
    if (k == "fit") {
        if (i + 1 >= w.size()) return why = "FIT takes an ISO 286 code: H7, g6, js9...", false;
        t = DimTolerance{};
        t.kind = DimTolerance::Fit;
        t.fit = w[i + 1];
        i += 2;
        if (i < w.size() && lowerOf(w[i]) == "values") t.values = true, ++i;
        FitDeviations probe;
        if (!isoFit(t.fit, 20, probe, &why)) return false;
        return true;
    }
    const bool limits = k == "limits";
    if (limits) ++i;
    if (i >= w.size()) return why = "LIMITS takes the deviations +0.1 -0.05", false;
    // "+0.1/-0.05" in one word.
    std::string first = w[i], second;
    if (const auto slash = first.find('/'); slash != std::string::npos) second = first.substr(slash + 1), first = first.substr(0, slash);
    double a = 0, b = 0;
    if (!number(first, a)) return why = "Not a tolerance: " + w[i], false;
    ++i;
    const bool explicitSign = !first.empty() && (first[0] == '+' || first[0] == '-');
    if (second.empty() && i < w.size() && (w[i][0] == '-' || w[i][0] == '+') && number(w[i], b)) second = w[i++];
    else if (!second.empty() && !number(second, b)) return why = "Not a tolerance: " + second, false;
    t = DimTolerance{};
    if (second.empty() && !limits && !explicitSign) { // a plain number: +/-
        t.kind = DimTolerance::Symmetric;
        t.upper = std::fabs(a), t.lower = -std::fabs(a);
        return true;
    }
    if (second.empty()) return why = "Give both deviations: +0.1 -0.05", false;
    t.kind = limits ? DimTolerance::Limits : DimTolerance::Deviation;
    t.upper = std::max(a, b), t.lower = std::min(a, b);
    return true;
}

std::string toleranceText(const DimTolerance& t) {
    switch (t.kind) {
    case DimTolerance::None: return "";
    case DimTolerance::Basic: return "basic";
    case DimTolerance::Fit: return "fit " + t.fit;
    case DimTolerance::Symmetric: return "+/-" + mm3(t.upper);
    default: return std::string(t.kind == DimTolerance::Limits ? "limits " : "") + "+" + mm3(t.upper) + " / " + mm3(t.lower);
    }
}

std::string frameText(const DrawingFrame& f) {
    std::string s = std::string(gdtName(f.symbol)) + " " + (f.diameter ? "dia " : "") + mm3(f.tolerance) + (f.material ? std::string(" (") + f.material + ")" : "");
    for (const std::string& x : f.datums) s += " | " + x;
    return s;
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
        for (std::size_t i = 0; i < d.dims.size(); ++i) {
            std::string line = "  dim " + std::to_string(i + 1) + ": " + dimKindName(d.dims[i].kind) + " in " + d.dims[i].view + " = " + mm(dimensionValue(d, d.dims[i])) + " mm";
            if (d.dims[i].tol.kind != DimTolerance::None) line += ", " + toleranceText(d.dims[i].tol);
            double hi, lo;
            if (dimensionLimits(d, d.dims[i], hi, lo)) line += " (" + mm3(lo) + " to " + mm3(hi) + ")";
            out.say(line);
        }
        for (std::size_t i = 0; i < d.frames.size(); ++i)
            out.say("  gdt " + std::to_string(i + 1) + ": " + frameText(d.frames[i]) +
                    (d.frames[i].dim >= 0 ? " under dim " + std::to_string(d.frames[i].dim + 1) : " in " + d.frames[i].view));
        for (const DrawingDatum& dt : d.datums) out.say("  datum " + dt.letter + " in " + dt.view);
        for (std::size_t i = 0; i < d.finishes.size(); ++i) out.say("  finish " + std::to_string(i + 1) + ": Ra " + mm3(d.finishes[i].ra) + " in " + d.finishes[i].view);
        if (!d.generalTolerance.empty()) out.say("  general tolerances " + d.generalTolerance);
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
            // Its dimensions go, and the frames under them; the others' dimension numbers close up.
            std::vector<int> renumber(d.dims.size(), -1);
            std::vector<DrawingDim> keptDims;
            for (std::size_t k = 0; k < d.dims.size(); ++k)
                if (lowerOf(d.dims[k].view) != lowerOf(name)) renumber[k] = static_cast<int>(keptDims.size()), keptDims.push_back(d.dims[k]);
            d.dims = std::move(keptDims);
            std::erase_if(d.frames, [&](DrawingFrame& f) {
                if (f.dim >= 0) return (f.dim = f.dim < static_cast<int>(renumber.size()) ? renumber[static_cast<std::size_t>(f.dim)] : -1) < 0;
                return lowerOf(f.view) == lowerOf(name);
            });
            std::erase_if(d.datums, [&](const DrawingDatum& x) { return lowerOf(x.view) == lowerOf(name); });
            std::erase_if(d.finishes, [&](const DrawingFinish& x) { return lowerOf(x.view) == lowerOf(name); });
            changed();
            out.say("Deleted view " + name + " and its dimensions and annotations");
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
            std::erase_if(d.frames, [](const DrawingFrame& f) { return f.dim >= 0; });
            changed();
            out.say("All dimensions removed");
            return out;
        }
        if (a.size() >= 3 && (lowerOf(a[1]) == "delete" || lowerOf(a[1]) == "remove")) {
            const auto n = parseNumber(a[2]);
            if (!n || *n < 1 || *n > static_cast<float>(d.dims.size())) return out.fail("No dimension " + a[2] + " (SHEET lists them)");
            const int gone = static_cast<int>(*n) - 1;
            d.dims.erase(d.dims.begin() + gone);
            std::erase_if(d.frames, [&](DrawingFrame& f) {
                if (f.dim == gone) return true;
                if (f.dim > gone) --f.dim;
                return false;
            });
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
            else if (w == "tol" || w == "tolerance") {
                std::string why;
                std::size_t j = i + 1;
                if (!parseTolerance(a, j, dm.tol, why)) return out.fail(why);
                i = j - 1;
            } else if (w == "fit" || w == "basic" || w == "limits") {
                std::string why;
                std::size_t j = i;
                if (!parseTolerance(a, j, dm.tol, why)) return out.fail(why);
                i = j - 1;
            }
        }
        if (dm.tol.kind == DimTolerance::Fit) {
            FitDeviations f;
            std::string why;
            if (!isoFit(dm.tol.fit, dimensionValue(d, dm), f, &why)) return out.fail(why);
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

    if (sub == "tol" || sub == "tolerance") {
        // SHEET TOL n 0.1 | +0.1 -0.05 | LIMITS +0.1 -0.05 | FIT H7 [VALUES] | BASIC | NONE
        // SHEET TOLERANCE ISO 2768-m[K] | NONE   (the general tolerance in the title block)
        if (a.size() >= 2 && (lowerOf(a[1]).rfind("iso", 0) == 0 || lowerOf(a[1]) == "general")) {
            std::string spec = rest(lowerOf(a[1]) == "general" ? 2 : 1);
            std::string up = toUpper(spec);
            std::erase_if(up, [](char c) { return c == ' '; });
            // ISO2768-M, ISO2768-MK, ISO 2768-m...
            const auto dash = up.find('-');
            if (up.rfind("ISO2768", 0) != 0 || dash == std::string::npos || dash + 1 >= up.size() ||
                std::string("FMCV").find(up[dash + 1]) == std::string::npos || (dash + 2 < up.size() && std::string("HKL").find(up[dash + 2]) == std::string::npos) ||
                dash + 3 < up.size())
                return out.fail("General tolerances: ISO 2768-f, -m, -c or -v, with H, K or L for the geometric class (ISO 2768-mK)");
            std::string cls = std::string(1, static_cast<char>(std::tolower(static_cast<unsigned char>(up[dash + 1])))) + (dash + 2 < up.size() ? std::string(1, up[dash + 2]) : "");
            d.generalTolerance = "ISO 2768-" + cls;
            changed();
            out.say("General tolerances " + d.generalTolerance + ": a 25 mm size +/-" + mm3(generalTolerance(cls[0], 25)) + " mm, 100 mm +/-" +
                    mm3(generalTolerance(cls[0], 100)) + " mm");
            return out;
        }
        if (a.size() >= 2 && lowerOf(a[1]) == "none" && a.size() == 2) {
            d.generalTolerance.clear();
            changed();
            out.say("No general tolerance");
            return out;
        }
        if (a.size() < 3) return out.fail("SHEET TOL n 0.1 | +0.1 -0.05 | LIMITS +0.1 -0.05 | FIT H7 [VALUES] | BASIC | NONE, or SHEET TOLERANCE ISO 2768-m");
        const auto n = parseNumber(a[1]);
        if (!n || *n < 1 || *n > static_cast<float>(d.dims.size())) return out.fail("No dimension " + a[1] + " (SHEET lists them)");
        DrawingDim& dm = d.dims[static_cast<std::size_t>(*n) - 1];
        DimTolerance t;
        std::string why;
        std::size_t i = 2;
        if (!parseTolerance(a, i, t, why)) return out.fail(why);
        if (t.kind == DimTolerance::Fit) {
            FitDeviations f;
            if (!isoFit(t.fit, dimensionValue(d, dm), f, &why)) return out.fail(why);
        }
        dm.tol = t;
        changed();
        std::string line = "Dimension " + a[1] + ": " + dimensionText(d, dm);
        double hi, lo;
        if (dimensionLimits(d, dm, hi, lo)) line += "  (" + mm3(lo) + " to " + mm3(hi) + " mm)";
        out.say(t.kind == DimTolerance::None ? "Dimension " + a[1] + ": no tolerance" : line);
        return out;
    }

    // Reads "view x,y,z" or "DIM n" (frames), then OFFSET dx,dy, from words[i...].
    auto attach = [&](std::size_t& i, std::string& viewName, DVec3& at, double& dx, double& dy, int* dimIndex, std::string& why) {
        bool placed = false;
        while (i < a.size()) {
            const std::string w = lowerOf(a[i]);
            if (dimIndex && (w == "dim" || w == "dimension") && i + 1 < a.size()) {
                const auto n = parseNumber(a[i + 1]);
                if (!n || *n < 1 || *n > static_cast<float>(d.dims.size())) return why = "No dimension " + a[i + 1], false;
                *dimIndex = static_cast<int>(*n) - 1;
                i += 2;
                placed = true;
            } else if ((w == "on" || w == "in") && i + 2 < a.size()) {
                const DrawingView* v = d.view(a[i + 1]);
                if (!v) return why = "No view named " + a[i + 1], false;
                const auto p = parseVec3(a[i + 2]);
                if (!p) return why = "The point is model x,y,z (mm)", false;
                viewName = v->name, at = DVec3(*p);
                i += 3;
                placed = true;
            } else if (w == "offset" && i + 1 < a.size()) {
                const auto p = parseVec3(a[i + 1]);
                if (!p) return why = "OFFSET takes dx,dy on the sheet (mm)", false;
                dx = p->x, dy = p->y;
                i += 2;
            } else
                break;
        }
        if (!placed) return why = dimIndex ? "Say where: ON view x,y,z or DIM n" : "Say where: ON view x,y,z", false;
        return true;
    };

    if (sub == "gdt" || sub == "frame" || sub == "fcf") {
        // SHEET GDT symbol tolerance [DIA] [MMC|LMC] [DATUMS A B C] ON view x,y,z [OFFSET dx,dy] | DIM n
        // SHEET GDT DELETE n | CLEAR
        if (a.size() >= 2 && lowerOf(a[1]) == "clear") {
            d.frames.clear();
            changed();
            out.say("All feature control frames removed");
            return out;
        }
        if (a.size() >= 3 && (lowerOf(a[1]) == "delete" || lowerOf(a[1]) == "remove")) {
            const auto n = parseNumber(a[2]);
            if (!n || *n < 1 || *n > static_cast<float>(d.frames.size())) return out.fail("No frame " + a[2] + " (SHEET lists them)");
            d.frames.erase(d.frames.begin() + static_cast<std::ptrdiff_t>(*n) - 1);
            changed();
            out.say("Feature control frame removed");
            return out;
        }
        const char* usage = "SHEET GDT symbol tolerance [DIA] [MMC|LMC] [DATUMS A B C] ON view x,y,z [OFFSET dx,dy] | DIM n  (symbols: straightness, "
                            "flatness, circularity, cylindricity, profile-line, profile-surface, angularity, perpendicularity, parallelism, "
                            "position, concentricity, symmetry, runout, total-runout)";
        if (a.size() < 3) return out.fail(usage);
        DrawingFrame f;
        if (!gdtFromName(a[1], &f.symbol)) return out.fail(std::string("Unknown characteristic ") + a[1] + ". " + usage);
        std::size_t i = 2;
        if (lowerOf(a[i]) == "dia" || lowerOf(a[i]) == "diameter") f.diameter = true, ++i;
        if (i >= a.size()) return out.fail(usage);
        std::string tolWord = a[i];
        if (tolWord.rfind("\xC3\x98", 0) == 0) f.diameter = true, tolWord = tolWord.substr(2);
        const auto tv = parseNumber(tolWord);
        if (!tv || !(*tv > 0)) return out.fail("The tolerance is a zone size in mm, over 0");
        f.tolerance = *tv;
        ++i;
        std::string why;
        while (i < a.size()) {
            const std::string w = lowerOf(a[i]);
            if (w == "dia" || w == "diameter") f.diameter = true, ++i;
            else if (w == "mmc" || w == "m") f.material = 'M', ++i;
            else if (w == "lmc" || w == "l") f.material = 'L', ++i;
            else if (w == "datum" || w == "datums") {
                ++i;
                while (i < a.size() && f.datums.size() < 3) {
                    std::string x = toUpper(a[i]);
                    // A letter, with an optional (M) or (L).
                    if (x.empty() || !std::isalpha(static_cast<unsigned char>(x[0])) || (x.size() > 1 && x != std::string(1, x[0]) + "(M)" && x != std::string(1, x[0]) + "(L)"))
                        break;
                    f.datums.push_back(x);
                    ++i;
                }
            } else
                break;
        }
        if (!attach(i, f.view, f.at, f.dx, f.dy, &f.dim, why)) return out.fail(why);
        if (i < a.size()) return out.fail("Not understood: " + a[i] + ". " + usage);
        if (!gdtTakesDatums(f.symbol) && !f.datums.empty()) return out.fail(std::string(gdtName(f.symbol)) + " is a form tolerance: it takes no datums");
        if (gdtNeedsDatum(f.symbol) && f.datums.empty()) return out.fail(std::string(gdtName(f.symbol)) + " is measured from a datum: DATUMS A [B [C]]");
        if (f.diameter && !gdtTakesDiameter(f.symbol)) return out.fail(std::string(gdtName(f.symbol)) + " has no cylindrical zone (no DIA)");
        if (f.material && (f.symbol == GdtSymbol::Flatness || f.symbol == GdtSymbol::Circularity || f.symbol == GdtSymbol::Cylindricity ||
                           f.symbol == GdtSymbol::LineProfile || f.symbol == GdtSymbol::SurfaceProfile || f.symbol == GdtSymbol::CircularRunout ||
                           f.symbol == GdtSymbol::TotalRunout || f.symbol == GdtSymbol::Concentricity || f.symbol == GdtSymbol::Symmetry))
            return out.fail(std::string("A material condition doesn't apply to ") + gdtName(f.symbol));
        for (std::size_t k = 0; k < f.datums.size(); ++k)
            for (std::size_t j = k + 1; j < f.datums.size(); ++j)
                if (f.datums[k][0] == f.datums[j][0]) return out.fail("Each datum once: " + f.datums[k]);
        d.frames.push_back(f);
        changed();
        out.say("Frame " + std::to_string(d.frames.size()) + ": " + frameText(f));
        for (const std::string& x : f.datums) {
            const std::string letter(1, x[0]);
            if (std::none_of(d.datums.begin(), d.datums.end(), [&](const DrawingDatum& dt) { return dt.letter == letter; }))
                out.say("  note: no datum " + letter + " on the drawing yet (SHEET DATUM " + letter + " ON view x,y,z)");
        }
        return out;
    }

    if (sub == "datum") {
        // SHEET DATUM letter ON view x,y,z [OFFSET dx,dy] | DELETE letter
        if (a.size() >= 3 && (lowerOf(a[1]) == "delete" || lowerOf(a[1]) == "remove")) {
            const std::string letter = toUpper(a[2]);
            const auto before = d.datums.size();
            std::erase_if(d.datums, [&](const DrawingDatum& x) { return x.letter == letter; });
            if (before == d.datums.size()) return out.fail("No datum " + letter);
            changed();
            out.say("Datum " + letter + " removed");
            return out;
        }
        if (a.size() < 2) return out.fail("SHEET DATUM letter ON view x,y,z [OFFSET dx,dy]");
        DrawingDatum dt;
        dt.letter = toUpper(a[1]);
        if (dt.letter.size() != 1 || !std::isalpha(static_cast<unsigned char>(dt.letter[0])) || dt.letter == "I" || dt.letter == "O" || dt.letter == "Q")
            return out.fail("A datum is one letter (not I, O or Q)");
        if (std::any_of(d.datums.begin(), d.datums.end(), [&](const DrawingDatum& x) { return x.letter == dt.letter; }))
            return out.fail("There is already a datum " + dt.letter);
        std::size_t i = 2;
        std::string why;
        if (!attach(i, dt.view, dt.at, dt.dx, dt.dy, nullptr, why)) return out.fail(why);
        d.datums.push_back(dt);
        changed();
        out.say("Datum " + dt.letter + " on " + dt.view);
        return out;
    }

    if (sub == "finish" || sub == "surface") {
        // SHEET FINISH [RA] 1.6 [MACHINED|UNMACHINED] ON view x,y,z [OFFSET dx,dy] | DELETE n
        if (a.size() >= 3 && (lowerOf(a[1]) == "delete" || lowerOf(a[1]) == "remove")) {
            const auto n = parseNumber(a[2]);
            if (!n || *n < 1 || *n > static_cast<float>(d.finishes.size())) return out.fail("No finish mark " + a[2]);
            d.finishes.erase(d.finishes.begin() + static_cast<std::ptrdiff_t>(*n) - 1);
            changed();
            out.say("Finish mark removed");
            return out;
        }
        DrawingFinish f;
        std::size_t i = 1;
        if (i < a.size() && lowerOf(a[i]) == "ra") ++i;
        const auto ra = i < a.size() ? parseNumber(a[i]) : std::nullopt;
        if (!ra || !(*ra > 0)) return out.fail("SHEET FINISH [RA] 1.6 [MACHINED|UNMACHINED] ON view x,y,z (Ra in micrometres)");
        f.ra = *ra;
        ++i;
        while (i < a.size()) {
            const std::string w = lowerOf(a[i]);
            if (w == "machined" || w == "removal") f.process = 1, ++i;
            else if (w == "unmachined" || w == "noremoval" || w == "asis") f.process = 2, ++i;
            else break;
        }
        std::string why;
        if (!attach(i, f.view, f.at, f.dx, f.dy, nullptr, why)) return out.fail(why);
        d.finishes.push_back(f);
        changed();
        out.say("Finish Ra " + mm3(f.ra) + " um on " + f.view);
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

    return out.fail("SHEET NEW | VIEW | SECTION | DIM | AUTODIM | TOL | GDT | DATUM | FINISH | TOLERANCE | TITLE | NOTE | SCALE | SIZE | PROJECTION | "
                    "EXPORT | SHOW | HIDE | DELETE");
}

} // namespace sicdm

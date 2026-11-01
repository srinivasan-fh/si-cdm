#include "core/VectorImport.hpp"

#include "core/Zip.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <unordered_set>

namespace sicdm {

namespace {

constexpr double kPiD = 3.14159265358979323846;

Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
Vec2 operator*(Vec2 a, float k) { return {a.x * k, a.y * k}; }
float length(Vec2 a) { return std::sqrt(a.x * a.x + a.y * a.y); }
constexpr double kFlatTol = 0.02; // mm: curves are flattened to within this

// A 2D affine map: x' = a x + c y + e, y' = b x + d y + f (PDF's order).
struct Affine {
    double a = 1, b = 0, c = 0, d = 1, e = 0, f = 0;
    Vec2 apply(double x, double y) const { return {static_cast<float>(a * x + c * y + e), static_cast<float>(b * x + d * y + f)}; }
    // this then o (o applied after this).
    Affine then(const Affine& o) const {
        return {a * o.a + b * o.c, a * o.b + b * o.d, c * o.a + d * o.c, c * o.b + d * o.d, e * o.a + f * o.c + o.e, e * o.b + f * o.d + o.f};
    }
    double scale() const { return std::sqrt(std::fabs(a * d - b * c)); }
    // Keeps circles circles: columns at right angles and of one length (a mirror is fine).
    bool similar() const {
        const double l1 = a * a + b * b, l2 = c * c + d * d;
        return std::fabs(a * c + b * d) <= 1e-9 * (l1 + l2) && std::fabs(l1 - l2) <= 1e-9 * (l1 + l2);
    }
};

// Builds paths: subpaths in output units (mm) with curves flattened.
struct PathBuilder {
    std::vector<VectorPath> out;
    std::vector<VectorPath> current;
    Vec2 start{}, at{};
    bool open = false;

    void moveTo(Vec2 p) {
        current.push_back({});
        current.back().pts.push_back(p);
        start = at = p;
        open = true;
    }
    void lineTo(Vec2 p) {
        if (!open) moveTo(at);
        if (current.back().pts.empty() || length(current.back().pts.back() - p) > 1e-6f) current.back().pts.push_back(p);
        at = p;
    }
    void cubicTo(Vec2 c1, Vec2 c2, Vec2 p) {
        if (!open) moveTo(at);
        const Vec2 p0 = at;
        const float poly = length(c1 - p0) + length(c2 - c1) + length(p - c2);
        const int n = std::clamp(static_cast<int>(std::ceil(std::sqrt(poly / kFlatTol) * 0.6)), 2, 96);
        for (int k = 1; k <= n; ++k) {
            const float t = static_cast<float>(k) / static_cast<float>(n), u = 1 - t;
            lineTo(p0 * (u * u * u) + c1 * (3 * u * u * t) + c2 * (3 * u * t * t) + p * (t * t * t));
        }
    }
    void close() {
        if (!open || current.empty()) return;
        current.back().closed = true;
        at = start;
        open = false;
    }
    // Paint: the subpaths are kept (filled ones are closed).
    void paint(bool fill) {
        for (VectorPath& p : current) {
            if (fill) p.closed = true;
            if (p.closed && p.pts.size() > 2 && length(p.pts.front() - p.pts.back()) < 1e-6f) p.pts.pop_back();
            if (p.pts.size() >= 2) out.push_back(std::move(p));
        }
        discard();
    }
    void discard() {
        current.clear();
        open = false;
    }
};

// Drops repeats (a path both filled and stroked by separate operators comes twice).
std::vector<VectorPath> unique(std::vector<VectorPath> in) {
    std::vector<VectorPath> out;
    std::unordered_set<std::string> seen;
    for (VectorPath& p : in) {
        std::string key(reinterpret_cast<const char*>(p.pts.data()), p.pts.size() * sizeof(Vec2));
        key += p.closed ? 'c' : 'o';
        key += p.circle ? std::to_string(p.radius) : "";
        if (seen.insert(key).second) out.push_back(std::move(p));
    }
    return out;
}

bool readFile(const std::string& path, std::string& bytes, std::string* error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        if (error) *error = "Can't read " + path;
        return false;
    }
    std::ostringstream s;
    s << f.rdbuf();
    bytes = s.str();
    return true;
}

// ---- PDF objects ---------------------------------------------------------------------

struct PdfObj;
using PdfPtr = std::shared_ptr<PdfObj>;

struct PdfObj {
    enum Type { Null, Bool, Num, Name, Str, Array, Dict, Ref, Op } type = Null;
    double num = 0;
    std::string str; // Name, Str, Op
    std::vector<PdfPtr> arr;
    std::map<std::string, PdfPtr> dict;
    int refNum = 0;
    std::string stream; // raw stream data of a stream object
    bool hasStream = false;

    const PdfObj* get(const std::string& k) const {
        const auto it = dict.find(k);
        return it == dict.end() ? nullptr : it->second.get();
    }
};

bool isWhite(char c) { return c == ' ' || c == '\n' || c == '\r' || c == '\t' || c == '\f' || c == '\0'; }
bool isDelim(char c) { return c == '(' || c == ')' || c == '<' || c == '>' || c == '[' || c == ']' || c == '{' || c == '}' || c == '/' || c == '%'; }

struct Lexer {
    const std::string& s;
    std::size_t i = 0;
    explicit Lexer(const std::string& text, std::size_t at = 0) : s(text), i(at) {}

    void skip() {
        while (i < s.size()) {
            if (isWhite(s[i])) ++i;
            else if (s[i] == '%') {
                while (i < s.size() && s[i] != '\n' && s[i] != '\r') ++i;
            } else break;
        }
    }
    bool done() {
        skip();
        return i >= s.size();
    }
    // One object; arrays and dictionaries whole. "R" after two numbers makes a reference.
    PdfPtr next(int depth = 0) {
        skip();
        auto o = std::make_shared<PdfObj>();
        if (i >= s.size() || depth > 64) return o;
        const char c = s[i];
        if (c == '/') {
            ++i;
            o->type = PdfObj::Name;
            while (i < s.size() && !isWhite(s[i]) && !isDelim(s[i])) {
                if (s[i] == '#' && i + 2 < s.size() && std::isxdigit(static_cast<unsigned char>(s[i + 1])) && std::isxdigit(static_cast<unsigned char>(s[i + 2]))) {
                    o->str += static_cast<char>(std::strtol(s.substr(i + 1, 2).c_str(), nullptr, 16));
                    i += 3;
                } else o->str += s[i++];
            }
            return o;
        }
        if (c == '(') {
            o->type = PdfObj::Str;
            int nest = 0;
            ++i;
            while (i < s.size()) {
                const char ch = s[i++];
                if (ch == '\\' && i < s.size()) {
                    o->str += s[i++];
                } else if (ch == '(') {
                    ++nest;
                    o->str += ch;
                } else if (ch == ')') {
                    if (nest-- == 0) break;
                    o->str += ch;
                } else o->str += ch;
            }
            return o;
        }
        if (c == '<' && i + 1 < s.size() && s[i + 1] == '<') {
            i += 2;
            o->type = PdfObj::Dict;
            while (!done()) {
                if (s.compare(i, 2, ">>") == 0) {
                    i += 2;
                    break;
                }
                const PdfPtr key = next(depth + 1);
                if (key->type != PdfObj::Name) {
                    if (key->type == PdfObj::Null && i < s.size()) ++i; // stray byte
                    continue;
                }
                o->dict[key->str] = next(depth + 1);
            }
            return o;
        }
        if (c == '<') {
            ++i;
            o->type = PdfObj::Str;
            std::string hex;
            while (i < s.size() && s[i] != '>') {
                if (std::isxdigit(static_cast<unsigned char>(s[i]))) hex += s[i];
                ++i;
            }
            ++i;
            if (hex.size() % 2) hex += '0';
            for (std::size_t k = 0; k < hex.size(); k += 2) o->str += static_cast<char>(std::strtol(hex.substr(k, 2).c_str(), nullptr, 16));
            return o;
        }
        if (c == '[') {
            ++i;
            o->type = PdfObj::Array;
            while (!done()) {
                if (s[i] == ']') {
                    ++i;
                    break;
                }
                o->arr.push_back(next(depth + 1));
                // "a b R" inside arrays.
                const std::size_t n = o->arr.size();
                if (n >= 3 && o->arr[n - 1]->type == PdfObj::Op && o->arr[n - 1]->str == "R" && o->arr[n - 2]->type == PdfObj::Num &&
                    o->arr[n - 3]->type == PdfObj::Num) {
                    auto r = std::make_shared<PdfObj>();
                    r->type = PdfObj::Ref;
                    r->refNum = static_cast<int>(o->arr[n - 3]->num);
                    o->arr.resize(n - 3);
                    o->arr.push_back(r);
                }
            }
            return o;
        }
        if (c == ']' || c == '>' || c == ')' || c == '{' || c == '}') {
            ++i;
            o->type = PdfObj::Op;
            o->str = std::string(1, c);
            return o;
        }
        // A number, a keyword or an operator.
        const std::size_t b = i;
        while (i < s.size() && !isWhite(s[i]) && !isDelim(s[i])) ++i;
        const std::string w = s.substr(b, i - b);
        if (w.empty()) {
            ++i;
            return o;
        }
        char* end = nullptr;
        const double v = std::strtod(w.c_str(), &end);
        if (end && *end == '\0' && (std::isdigit(static_cast<unsigned char>(w[0])) || w[0] == '-' || w[0] == '+' || w[0] == '.')) {
            // "n g R": a reference.
            const std::size_t save = i;
            skip();
            const std::size_t b2 = i;
            while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) ++i;
            if (i > b2) {
                skip();
                if (i < s.size() && s[i] == 'R' && (i + 1 >= s.size() || isWhite(s[i + 1]) || isDelim(s[i + 1]))) {
                    ++i;
                    o->type = PdfObj::Ref;
                    o->refNum = static_cast<int>(v);
                    return o;
                }
            }
            i = save;
            o->type = PdfObj::Num;
            o->num = v;
            return o;
        }
        if (w == "true" || w == "false") {
            o->type = PdfObj::Bool;
            o->num = w == "true";
            return o;
        }
        if (w == "null") return o;
        o->type = PdfObj::Op;
        o->str = w;
        return o;
    }
};

std::string zlibInflate(const std::string& data, bool& ok) {
    std::string out;
    ok = false;
    if (data.size() < 2) return out;
    const std::size_t skip = ((static_cast<unsigned char>(data[0]) & 0x0F) == 8) ? 2 : 0; // a zlib header, or raw
    ok = inflateRaw(reinterpret_cast<const std::uint8_t*>(data.data()) + skip, data.size() - skip, out, std::size_t{1} << 30);
    return out;
}

std::string asciiHex(const std::string& in) {
    std::string hex, out;
    for (char c : in) {
        if (c == '>') break;
        if (std::isxdigit(static_cast<unsigned char>(c))) hex += c;
    }
    if (hex.size() % 2) hex += '0';
    for (std::size_t k = 0; k < hex.size(); k += 2) out += static_cast<char>(std::strtol(hex.substr(k, 2).c_str(), nullptr, 16));
    return out;
}

std::string ascii85(const std::string& in) {
    std::string out;
    std::uint32_t tuple = 0;
    int n = 0;
    for (std::size_t k = 0; k < in.size(); ++k) {
        const char c = in[k];
        if (c == '~') break;
        if (isWhite(c)) continue;
        if (c == 'z' && n == 0) {
            out.append(4, '\0');
            continue;
        }
        if (c < '!' || c > 'u') continue;
        tuple = tuple * 85 + static_cast<std::uint32_t>(c - '!');
        if (++n == 5) {
            for (int b = 3; b >= 0; --b) out += static_cast<char>((tuple >> (8 * b)) & 0xFF);
            tuple = 0, n = 0;
        }
    }
    if (n > 1) {
        for (int k = n; k < 5; ++k) tuple = tuple * 85 + 84;
        for (int b = 3; b >= 5 - n; --b) out += static_cast<char>((tuple >> (8 * b)) & 0xFF);
    }
    return out;
}

struct PdfFile {
    std::map<int, PdfPtr> objects;
    std::vector<std::string> warnings;
    std::string version;

    const PdfObj* resolve(const PdfObj* o, int depth = 0) const {
        while (o && o->type == PdfObj::Ref && depth++ < 32) {
            const auto it = objects.find(o->refNum);
            o = it == objects.end() ? nullptr : it->second.get();
        }
        return o;
    }
    const PdfObj* get(const PdfObj* dict, const std::string& k) const { return dict ? resolve(dict->get(k)) : nullptr; }

    // A stream's bytes through its filters; false (with a warning) for a filter not read.
    bool decode(const PdfObj& o, std::string& data) {
        data = o.stream;
        const PdfObj* f = get(&o, "Filter");
        std::vector<std::string> filters;
        if (f && f->type == PdfObj::Name) filters.push_back(f->str);
        if (f && f->type == PdfObj::Array)
            for (const PdfPtr& p : f->arr)
                if (const PdfObj* r = resolve(p.get()); r && r->type == PdfObj::Name) filters.push_back(r->str);
        for (const std::string& name : filters) {
            if (name == "FlateDecode" || name == "Fl") {
                bool ok = false;
                data = zlibInflate(data, ok);
                if (!ok) {
                    warnings.push_back("a damaged compressed stream was skipped");
                    return false;
                }
                const PdfObj* parms = get(&o, "DecodeParms");
                if (const PdfObj* pr = get(parms, "Predictor"); pr && pr->num >= 10) {
                    // PNG predictors (rows of `Columns` bytes, each with a filter byte): Up and None.
                    const PdfObj* colsObj = get(parms, "Columns");
                    const std::size_t cols = colsObj ? static_cast<std::size_t>(colsObj->num) : 1;
                    std::string rows;
                    std::string prev(cols, '\0');
                    for (std::size_t at = 0; at + cols + 1 <= data.size(); at += cols + 1) {
                        const int type = static_cast<unsigned char>(data[at]);
                        std::string row = data.substr(at + 1, cols);
                        for (std::size_t k = 0; k < cols; ++k) {
                            const unsigned char left = k ? static_cast<unsigned char>(row[k - 1]) : 0, up = static_cast<unsigned char>(prev[k]);
                            if (type == 1) row[k] = static_cast<char>(row[k] + left);
                            else if (type == 2) row[k] = static_cast<char>(row[k] + up);
                            else if (type == 3) row[k] = static_cast<char>(row[k] + (left + up) / 2);
                            else if (type == 4) {
                                const unsigned char ul = k ? static_cast<unsigned char>(prev[k - 1]) : 0;
                                const int p = left + up - ul, pa = std::abs(p - left), pb = std::abs(p - up), pc = std::abs(p - ul);
                                row[k] = static_cast<char>(row[k] + (pa <= pb && pa <= pc ? left : pb <= pc ? up : ul));
                            }
                        }
                        rows += row;
                        prev = row;
                    }
                    data = rows;
                }
            } else if (name == "ASCIIHexDecode" || name == "AHx") {
                data = asciiHex(data);
            } else if (name == "ASCII85Decode" || name == "A85") {
                data = ascii85(data);
            } else {
                warnings.push_back("a stream in " + name + " was skipped (not read)");
                return false;
            }
        }
        return true;
    }

    bool parse(const std::string& b, std::string* error) {
        if (b.compare(0, 5, "%PDF-") != 0 && b.find("%PDF-") == std::string::npos) {
            if (error) *error = "Not a PDF file";
            return false;
        }
        const std::size_t vp = b.find("%PDF-");
        version = b.substr(vp + 5, 3);
        // Every "n g obj" in the file, later ones (incremental updates) winning.
        for (std::size_t at = b.find("obj"); at != std::string::npos; at = b.find("obj", at + 3)) {
            if (at > 0 && !isWhite(b[at - 1])) continue;
            if (at + 3 < b.size() && !isWhite(b[at + 3]) && !isDelim(b[at + 3])) continue;
            // Back over "n g ".
            std::size_t k = at;
            while (k > 0 && isWhite(b[k - 1])) --k;
            std::size_t g = k;
            while (g > 0 && std::isdigit(static_cast<unsigned char>(b[g - 1]))) --g;
            if (g == k) continue;
            std::size_t k2 = g;
            while (k2 > 0 && isWhite(b[k2 - 1])) --k2;
            std::size_t n = k2;
            while (n > 0 && std::isdigit(static_cast<unsigned char>(b[n - 1]))) --n;
            if (n == k2 || k2 == g) continue;
            const int num = std::atoi(b.substr(n, k2 - n).c_str());
            Lexer lx(b, at + 3);
            PdfPtr o = lx.next();
            lx.skip();
            if (b.compare(lx.i, 6, "stream") == 0) {
                std::size_t d = lx.i + 6;
                if (d < b.size() && b[d] == '\r') ++d;
                if (d < b.size() && b[d] == '\n') ++d;
                std::size_t end = std::string::npos;
                if (const PdfObj* len = o->get("Length"); len && len->type == PdfObj::Num) {
                    const std::size_t e = d + static_cast<std::size_t>(len->num);
                    const std::size_t es = b.find("endstream", e);
                    if (e <= b.size() && es != std::string::npos && es - e < 4) end = e;
                }
                if (end == std::string::npos) { // the length is elsewhere (a reference) or wrong
                    const std::size_t es = b.find("endstream", d);
                    if (es == std::string::npos) continue;
                    end = es;
                    while (end > d && (b[end - 1] == '\n' || b[end - 1] == '\r')) --end;
                    // A referenced length is right when it is there.
                    if (const PdfObj* len = o->get("Length"); len && len->type == PdfObj::Ref) o->refNum = len->refNum;
                }
                o->stream = b.substr(d, end - d);
                o->hasStream = true;
            }
            objects[num] = o;
        }
        // Streams whose /Length is a reference: cut to that length now it can be read.
        for (auto& [num, o] : objects) {
            if (!o->hasStream) continue;
            if (const PdfObj* len = resolve(o->get("Length")); len && len->type == PdfObj::Num && len->num >= 0 &&
                                                               static_cast<std::size_t>(len->num) <= o->stream.size())
                o->stream.resize(static_cast<std::size_t>(len->num));
        }
        // Object streams (PDF 1.5): the objects packed inside.
        std::vector<std::pair<int, PdfPtr>> packed;
        for (auto& [num, o] : objects) {
            const PdfObj* type = get(o.get(), "Type");
            if (!o->hasStream || !type || type->type != PdfObj::Name || type->str != "ObjStm") continue;
            std::string data;
            if (!decode(*o, data)) continue;
            const PdfObj* nObj = get(o.get(), "N");
            const PdfObj* first = get(o.get(), "First");
            if (!nObj || !first) continue;
            Lexer header(data);
            std::vector<std::pair<int, std::size_t>> index;
            for (int k = 0; k < static_cast<int>(nObj->num); ++k) {
                const PdfPtr a = header.next(), c = header.next();
                if (a->type != PdfObj::Num || c->type != PdfObj::Num) break;
                index.push_back({static_cast<int>(a->num), static_cast<std::size_t>(c->num)});
            }
            for (const auto& [objNum, off] : index) {
                Lexer lx(data, static_cast<std::size_t>(first->num) + off);
                packed.push_back({objNum, lx.next()});
            }
        }
        for (auto& [num, o] : packed)
            if (!objects.count(num)) objects[num] = o;
        if (objects.empty()) {
            if (error) *error = "No objects found: a damaged PDF";
            return false;
        }
        return true;
    }

    // The pages in order: the catalog's page tree, else every /Page object.
    std::vector<const PdfObj*> pages() const {
        std::vector<const PdfObj*> out;
        const PdfObj* catalog = nullptr;
        for (const auto& [num, o] : objects)
            if (const PdfObj* t = get(o.get(), "Type"); t && t->type == PdfObj::Name && t->str == "Catalog") catalog = o.get();
        std::set<const PdfObj*> seen;
        std::function<void(const PdfObj*, int)> walk = [&](const PdfObj* node, int depth) {
            if (!node || depth > 64 || !seen.insert(node).second) return;
            const PdfObj* t = get(node, "Type");
            if (t && t->type == PdfObj::Name && t->str == "Page") {
                out.push_back(node);
                return;
            }
            if (const PdfObj* kids = get(node, "Kids"); kids && kids->type == PdfObj::Array)
                for (const PdfPtr& k : kids->arr) walk(resolve(k.get()), depth + 1);
        };
        if (catalog) walk(get(catalog, "Pages"), 0);
        if (out.empty())
            for (const auto& [num, o] : objects)
                if (const PdfObj* t = get(o.get(), "Type"); t && t->type == PdfObj::Name && t->str == "Page") out.push_back(o.get());
        return out;
    }

    // A page's attribute, inherited from the page tree.
    const PdfObj* inherited(const PdfObj* page, const std::string& key) const {
        for (int depth = 0; page && depth < 64; ++depth) {
            if (const PdfObj* v = get(page, key)) return v;
            page = get(page, "Parent");
        }
        return nullptr;
    }
};

double numAt(const std::vector<PdfPtr>& st, std::size_t fromEnd) {
    if (fromEnd >= st.size()) return 0;
    const PdfObj& o = *st[st.size() - 1 - fromEnd];
    return o.type == PdfObj::Num ? o.num : 0;
}

// Runs a content stream: paths in page points through the CTM, then to mm.
void runContent(PdfFile& pdf, const std::string& content, const PdfObj* resources, Affine ctm, const Affine& toMm, PathBuilder& pb, int depth) {
    if (depth > 12) return;
    std::vector<Affine> stack;
    std::vector<PdfPtr> operands;
    Lexer lx(content);
    auto pt = [&](double x, double y) {
        const Vec2 p = ctm.apply(x, y);
        return toMm.apply(p.x, p.y);
    };
    while (!lx.done()) {
        PdfPtr t = lx.next();
        if (t->type != PdfObj::Op) {
            operands.push_back(std::move(t));
            if (operands.size() > 64) operands.erase(operands.begin());
            continue;
        }
        const std::string& op = t->str;
        if (op == "q") stack.push_back(ctm);
        else if (op == "Q") {
            if (!stack.empty()) ctm = stack.back(), stack.pop_back();
        } else if (op == "cm" && operands.size() >= 6) {
            const Affine m{numAt(operands, 5), numAt(operands, 4), numAt(operands, 3), numAt(operands, 2), numAt(operands, 1), numAt(operands, 0)};
            ctm = m.then(ctm);
        } else if (op == "m" && operands.size() >= 2) pb.moveTo(pt(numAt(operands, 1), numAt(operands, 0)));
        else if (op == "l" && operands.size() >= 2) pb.lineTo(pt(numAt(operands, 1), numAt(operands, 0)));
        else if (op == "c" && operands.size() >= 6)
            pb.cubicTo(pt(numAt(operands, 5), numAt(operands, 4)), pt(numAt(operands, 3), numAt(operands, 2)), pt(numAt(operands, 1), numAt(operands, 0)));
        else if (op == "v" && operands.size() >= 4)
            pb.cubicTo(pb.at, pt(numAt(operands, 3), numAt(operands, 2)), pt(numAt(operands, 1), numAt(operands, 0)));
        else if (op == "y" && operands.size() >= 4) {
            const Vec2 p = pt(numAt(operands, 1), numAt(operands, 0));
            pb.cubicTo(pt(numAt(operands, 3), numAt(operands, 2)), p, p);
        } else if (op == "h") pb.close();
        else if (op == "re" && operands.size() >= 4) {
            const double x = numAt(operands, 3), y = numAt(operands, 2), w = numAt(operands, 1), h = numAt(operands, 0);
            pb.moveTo(pt(x, y));
            pb.lineTo(pt(x + w, y));
            pb.lineTo(pt(x + w, y + h));
            pb.lineTo(pt(x, y + h));
            pb.close();
        } else if (op == "S" || op == "s") {
            if (op == "s") pb.close();
            pb.paint(false);
        } else if (op == "f" || op == "F" || op == "f*" || op == "B" || op == "B*" || op == "b" || op == "b*") {
            pb.paint(true);
        } else if (op == "n") {
            pb.discard(); // a clipping path (W n) or nothing drawn
        } else if (op == "BI") {
            // An inline image: its data runs to "EI".
            const std::size_t id = content.find("ID", lx.i);
            const std::size_t ei = id == std::string::npos ? std::string::npos : content.find("EI", id + 3);
            lx.i = ei == std::string::npos ? content.size() : ei + 2;
        } else if (op == "Do" && !operands.empty() && operands.back()->type == PdfObj::Name) {
            const PdfObj* xo = pdf.get(pdf.get(resources, "XObject"), operands.back()->str);
            const PdfObj* sub = pdf.get(xo, "Subtype");
            if (xo && xo->hasStream && sub && sub->type == PdfObj::Name && sub->str == "Form") {
                std::string data;
                if (pdf.decode(*xo, data)) {
                    Affine m;
                    if (const PdfObj* mat = pdf.get(xo, "Matrix"); mat && mat->type == PdfObj::Array && mat->arr.size() == 6) {
                        double v[6];
                        for (int k = 0; k < 6; ++k) {
                            const PdfObj* e = pdf.resolve(mat->arr[static_cast<std::size_t>(k)].get());
                            v[k] = e && e->type == PdfObj::Num ? e->num : 0;
                        }
                        m = {v[0], v[1], v[2], v[3], v[4], v[5]};
                    }
                    const PdfObj* res = pdf.get(xo, "Resources");
                    runContent(pdf, data, res ? res : resources, m.then(ctm), toMm, pb, depth + 1);
                }
            }
        }
        operands.clear();
    }
}

// ---- SVG -----------------------------------------------------------------------------

struct XmlElement {
    std::string name;
    std::map<std::string, std::string> attrs;
    bool closing = false, selfClosing = false;
};

// The next element tag from `i` (comments, declarations and text skipped).
bool nextTag(const std::string& s, std::size_t& i, XmlElement& e) {
    while (true) {
        i = s.find('<', i);
        if (i == std::string::npos) return false;
        if (s.compare(i, 4, "<!--") == 0) {
            const std::size_t end = s.find("-->", i);
            if (end == std::string::npos) return false;
            i = end + 3;
            continue;
        }
        if (s.compare(i, 2, "<?") == 0 || s.compare(i, 2, "<!") == 0) {
            const std::size_t end = s.find('>', i);
            if (end == std::string::npos) return false;
            i = end + 1;
            continue;
        }
        break;
    }
    e = {};
    ++i;
    if (i < s.size() && s[i] == '/') e.closing = true, ++i;
    while (i < s.size() && !isWhite(s[i]) && s[i] != '>' && s[i] != '/') e.name += s[i++];
    if (const std::size_t colon = e.name.find(':'); colon != std::string::npos) e.name = e.name.substr(colon + 1); // svg:path
    while (i < s.size()) {
        while (i < s.size() && isWhite(s[i])) ++i;
        if (i >= s.size()) return false;
        if (s[i] == '>') {
            ++i;
            return true;
        }
        if (s[i] == '/') {
            e.selfClosing = true;
            ++i;
            continue;
        }
        std::string key;
        while (i < s.size() && !isWhite(s[i]) && s[i] != '=' && s[i] != '>' && s[i] != '/') key += s[i++];
        while (i < s.size() && isWhite(s[i])) ++i;
        std::string value;
        if (i < s.size() && s[i] == '=') {
            ++i;
            while (i < s.size() && isWhite(s[i])) ++i;
            if (i < s.size() && (s[i] == '"' || s[i] == '\'')) {
                const char q = s[i++];
                const std::size_t end = s.find(q, i);
                if (end == std::string::npos) return false;
                value = s.substr(i, end - i);
                i = end + 1;
            }
        }
        if (!key.empty()) e.attrs[key] = value;
    }
    return false;
}

// Numbers in a list ("10,20 30-5e1": separators, signs and exponents).
struct NumReader {
    const std::string& s;
    std::size_t i = 0;
    explicit NumReader(const std::string& t) : s(t) {}
    void sep() {
        while (i < s.size() && (isWhite(s[i]) || s[i] == ',')) ++i;
    }
    bool number(double& v) {
        sep();
        if (i >= s.size()) return false;
        const char* b = s.c_str() + i;
        char* end = nullptr;
        v = std::strtod(b, &end);
        if (end == b) return false;
        i += static_cast<std::size_t>(end - b);
        return true;
    }
    // An arc's flag: one digit, which may run into the next number ("a1 1 0 01 10 20").
    bool flag(bool& f) {
        sep();
        if (i >= s.size() || (s[i] != '0' && s[i] != '1')) return false;
        f = s[i++] == '1';
        return true;
    }
};

Affine parseTransform(const std::string& t) {
    Affine m;
    std::size_t i = 0;
    while (i < t.size()) {
        while (i < t.size() && (isWhite(t[i]) || t[i] == ',')) ++i;
        std::string name;
        while (i < t.size() && std::isalpha(static_cast<unsigned char>(t[i]))) name += t[i++];
        const std::size_t open = t.find('(', i), close = t.find(')', i);
        if (name.empty() || open == std::string::npos || close == std::string::npos) break;
        const std::string args = t.substr(open + 1, close - open - 1);
        i = close + 1;
        NumReader r(args);
        std::vector<double> v;
        for (double x; r.number(x);) v.push_back(x);
        Affine a;
        if (name == "matrix" && v.size() == 6) a = {v[0], v[1], v[2], v[3], v[4], v[5]};
        else if (name == "translate" && !v.empty()) a.e = v[0], a.f = v.size() > 1 ? v[1] : 0;
        else if (name == "scale" && !v.empty()) a.a = v[0], a.d = v.size() > 1 ? v[1] : v[0];
        else if (name == "rotate" && !v.empty()) {
            const double r0 = v[0] * kPiD / 180, c = std::cos(r0), s = std::sin(r0);
            a = {c, s, -s, c, 0, 0};
            if (v.size() == 3) a = Affine{1, 0, 0, 1, -v[1], -v[2]}.then(a).then(Affine{1, 0, 0, 1, v[1], v[2]});
        } else if (name == "skewX" && !v.empty()) a.c = std::tan(v[0] * kPiD / 180);
        else if (name == "skewY" && !v.empty()) a.b = std::tan(v[0] * kPiD / 180);
        m = a.then(m); // the list applies right to left
    }
    return m;
}

// A length attribute in mm (unitless: user units, `user` mm each).
std::optional<double> svgLength(const std::string& v, double user) {
    if (v.empty()) return std::nullopt;
    char* end = nullptr;
    const double x = std::strtod(v.c_str(), &end);
    if (end == v.c_str()) return std::nullopt;
    std::string unit(end);
    unit.erase(std::remove_if(unit.begin(), unit.end(), [](char c) { return isWhite(c); }), unit.end());
    if (unit == "mm") return x;
    if (unit == "cm") return x * 10;
    if (unit == "in") return x * 25.4;
    if (unit == "pt") return x * 25.4 / 72;
    if (unit == "pc") return x * 25.4 / 6;
    if (unit == "px" || unit.empty()) return x * user;
    return std::nullopt; // % and em: not a size
}

void svgPath(const std::string& d, const std::function<Vec2(double, double)>& T, PathBuilder& pb) {
    NumReader r(d);
    char cmd = 0;
    double cx = 0, cy = 0, sx = 0, sy = 0; // current point and subpath start (user units)
    double lcx = 0, lcy = 0;               // the last control point (for S and T)
    char last = 0;
    auto cubic = [&](double x1, double y1, double x2, double y2, double x, double y) {
        pb.cubicTo(T(x1, y1), T(x2, y2), T(x, y));
        lcx = x2, lcy = y2;
        cx = x, cy = y;
    };
    while (true) {
        r.sep();
        if (r.i >= d.size()) break;
        if (std::isalpha(static_cast<unsigned char>(d[r.i])) && d[r.i] != 'e' && d[r.i] != 'E') cmd = d[r.i++];
        else if (!cmd) break;
        const bool rel = std::islower(static_cast<unsigned char>(cmd));
        const char C = static_cast<char>(std::toupper(static_cast<unsigned char>(cmd)));
        double v[7];
        auto get = [&](int n) {
            for (int k = 0; k < n; ++k)
                if (!r.number(v[k])) return false;
            return true;
        };
        bool ok = true;
        switch (C) {
        case 'M':
            if ((ok = get(2))) {
                cx = (rel ? cx : 0) + v[0], cy = (rel ? cy : 0) + v[1];
                pb.moveTo(T(cx, cy));
                sx = cx, sy = cy;
                cmd = rel ? 'l' : 'L'; // later pairs are lines
            }
            break;
        case 'L':
            if ((ok = get(2))) {
                cx = (rel ? cx : 0) + v[0], cy = (rel ? cy : 0) + v[1];
                pb.lineTo(T(cx, cy));
            }
            break;
        case 'H':
            if ((ok = get(1))) {
                cx = (rel ? cx : 0) + v[0];
                pb.lineTo(T(cx, cy));
            }
            break;
        case 'V':
            if ((ok = get(1))) {
                cy = (rel ? cy : 0) + v[0];
                pb.lineTo(T(cx, cy));
            }
            break;
        case 'C':
            if ((ok = get(6))) {
                const double ox = rel ? cx : 0, oy = rel ? cy : 0;
                cubic(ox + v[0], oy + v[1], ox + v[2], oy + v[3], ox + v[4], oy + v[5]);
            }
            break;
        case 'S':
            if ((ok = get(4))) {
                const double ox = rel ? cx : 0, oy = rel ? cy : 0;
                const bool after = last == 'C' || last == 'S';
                const double x1 = after ? 2 * cx - lcx : cx, y1 = after ? 2 * cy - lcy : cy;
                cubic(x1, y1, ox + v[0], oy + v[1], ox + v[2], oy + v[3]);
            }
            break;
        case 'Q':
        case 'T': {
            double qx, qy, x, y;
            const double ox = rel ? cx : 0, oy = rel ? cy : 0;
            if (C == 'Q') {
                if (!(ok = get(4))) break;
                qx = ox + v[0], qy = oy + v[1], x = ox + v[2], y = oy + v[3];
            } else {
                if (!(ok = get(2))) break;
                const bool after = last == 'Q' || last == 'T';
                qx = after ? 2 * cx - lcx : cx, qy = after ? 2 * cy - lcy : cy;
                x = ox + v[0], y = oy + v[1];
            }
            pb.cubicTo(T(cx + 2.0 / 3 * (qx - cx), cy + 2.0 / 3 * (qy - cy)), T(x + 2.0 / 3 * (qx - x), y + 2.0 / 3 * (qy - y)), T(x, y));
            lcx = qx, lcy = qy;
            cx = x, cy = y;
            break;
        }
        case 'A': {
            bool large = false, sweep = false;
            double rx, ry, rot;
            if (!(ok = r.number(rx) && r.number(ry) && r.number(rot) && r.flag(large) && r.flag(sweep) && r.number(v[0]) && r.number(v[1]))) break;
            const double x = (rel ? cx : 0) + v[0], y = (rel ? cy : 0) + v[1];
            rx = std::fabs(rx), ry = std::fabs(ry);
            if (rx < 1e-12 || ry < 1e-12 || (x == cx && y == cy)) {
                pb.lineTo(T(x, y));
                cx = x, cy = y;
                break;
            }
            // Endpoint to centre form (SVG 1.1, appendix F.6).
            const double phi = rot * kPiD / 180, cp = std::cos(phi), sp = std::sin(phi);
            const double dx = (cx - x) / 2, dy = (cy - y) / 2;
            const double x1 = cp * dx + sp * dy, y1 = -sp * dx + cp * dy;
            const double lambda = x1 * x1 / (rx * rx) + y1 * y1 / (ry * ry);
            if (lambda > 1) rx *= std::sqrt(lambda), ry *= std::sqrt(lambda);
            double num = rx * rx * ry * ry - rx * rx * y1 * y1 - ry * ry * x1 * x1;
            const double den = rx * rx * y1 * y1 + ry * ry * x1 * x1;
            double k = den > 0 ? std::sqrt(std::max(num, 0.0) / den) : 0;
            if (large == sweep) k = -k;
            const double cxp = k * rx * y1 / ry, cyp = -k * ry * x1 / rx;
            const double ccx = cp * cxp - sp * cyp + (cx + x) / 2, ccy = sp * cxp + cp * cyp + (cy + y) / 2;
            auto angle = [](double ux, double uy, double vx, double vy) {
                const double a = std::atan2(ux * vy - uy * vx, ux * vx + uy * vy);
                return a;
            };
            const double t1 = angle(1, 0, (x1 - cxp) / rx, (y1 - cyp) / ry);
            double dt = angle((x1 - cxp) / rx, (y1 - cyp) / ry, (-x1 - cxp) / rx, (-y1 - cyp) / ry);
            if (!sweep && dt > 0) dt -= 2 * kPiD;
            if (sweep && dt < 0) dt += 2 * kPiD;
            const int n = std::clamp(static_cast<int>(std::ceil(std::fabs(dt) * 18 / kPiD)), 2, 128);
            for (int s = 1; s <= n; ++s) {
                const double t = t1 + dt * s / n;
                const double ex = rx * std::cos(t), ey = ry * std::sin(t);
                pb.lineTo(T(cp * ex - sp * ey + ccx, sp * ex + cp * ey + ccy));
            }
            cx = x, cy = y;
            break;
        }
        case 'Z':
            pb.close();
            cx = sx, cy = sy;
            break;
        default: ok = false; break;
        }
        if (!ok) break;
        last = C;
    }
}

} // namespace

std::vector<VectorPath> parsePdf(const std::string& bytes, int page, double scale, VectorImportReport* report, std::string* error) {
    PdfFile pdf;
    if (!pdf.parse(bytes, error)) return {};
    const std::vector<const PdfObj*> pages = pdf.pages();
    if (report) report->format = "PDF " + pdf.version, report->pages = static_cast<int>(pages.size());
    if (pages.empty()) {
        if (error) *error = "The PDF has no pages";
        return {};
    }
    if (page < 1 || page > static_cast<int>(pages.size())) {
        if (error) *error = "The PDF has " + std::to_string(pages.size()) + " page" + (pages.size() == 1 ? "" : "s") + "; PAGE " + std::to_string(page) + " isn't one";
        return {};
    }
    const PdfObj* pg = pages[static_cast<std::size_t>(page - 1)];
    const double k = 25.4 / 72.0 * (scale > 0 ? scale : 1.0);
    Affine toMm{k, 0, 0, k, 0, 0};
    double box[4] = {0, 0, 612, 792};
    if (const PdfObj* mb = pdf.resolve(pdf.inherited(pg, "MediaBox")); mb && mb->type == PdfObj::Array && mb->arr.size() == 4)
        for (int i = 0; i < 4; ++i)
            if (const PdfObj* e = pdf.resolve(mb->arr[static_cast<std::size_t>(i)].get()); e && e->type == PdfObj::Num) box[i] = e->num;
    toMm.e = -box[0] * k, toMm.f = -box[1] * k; // the page's lower-left corner at the origin
    if (report) report->page = page, report->size = {static_cast<float>((box[2] - box[0]) * k), static_cast<float>((box[3] - box[1]) * k)};
    // The page's content: one stream or an array of them, run as one.
    std::string content;
    const PdfObj* c = pdf.get(pg, "Contents");
    std::vector<const PdfObj*> parts;
    if (c && c->type == PdfObj::Array)
        for (const PdfPtr& p : c->arr) parts.push_back(pdf.resolve(p.get()));
    else if (c) parts.push_back(c);
    for (const PdfObj* p : parts) {
        std::string data;
        if (p && p->hasStream && pdf.decode(*p, data)) content += data + "\n";
    }
    PathBuilder pb;
    runContent(pdf, content, pdf.resolve(pdf.inherited(pg, "Resources")), Affine{}, toMm, pb, 0);
    if (report)
        for (const std::string& w : pdf.warnings)
            if (std::find(report->warnings.begin(), report->warnings.end(), w) == report->warnings.end()) report->warnings.push_back(w);
    std::vector<VectorPath> out = unique(std::move(pb.out));
    if (out.empty() && error) *error = "No line work on page " + std::to_string(page) + " (a scanned page is a picture: import it as an image)";
    return out;
}

std::vector<VectorPath> parseSvg(const std::string& text, double scale, VectorImportReport* report, std::string* error) {
    if (report) report->format = "SVG";
    std::size_t i = 0;
    XmlElement e;
    // The root: its size and viewBox give the user unit in mm.
    double userMm = 25.4 / 96.0, height = 0, vbX = 0, vbY = 0;
    bool found = false;
    while (nextTag(text, i, e))
        if (!e.closing && e.name == "svg") {
            found = true;
            break;
        }
    if (!found) {
        if (error) *error = "Not an SVG file";
        return {};
    }
    const auto w = svgLength(e.attrs["width"], 25.4 / 96.0), h = svgLength(e.attrs["height"], 25.4 / 96.0);
    if (const std::string vb = e.attrs["viewBox"]; !vb.empty()) {
        NumReader r(vb);
        double v[4];
        if (r.number(v[0]) && r.number(v[1]) && r.number(v[2]) && r.number(v[3]) && v[2] > 0 && v[3] > 0) {
            vbX = v[0], vbY = v[1];
            userMm = w ? *w / v[2] : h ? *h / v[3] : 25.4 / 96.0;
            height = v[3];
        }
    } else if (h) {
        height = *h / userMm;
    }
    const double k = userMm * (scale > 0 ? scale : 1.0);
    if (report) report->size = {static_cast<float>(w ? *w * (scale > 0 ? scale : 1) : 0), static_cast<float>(h ? *h * (scale > 0 ? scale : 1) : 0)};
    // User units to mm with y up (the bottom of the document at y 0).
    const Affine toMm{k, 0, 0, -k, -vbX * k, (height + vbY) * k};
    std::vector<Affine> stack{Affine{}};
    PathBuilder pb;
    auto num = [](const std::map<std::string, std::string>& a, const char* key) {
        const auto it = a.find(key);
        return it == a.end() ? 0.0 : std::strtod(it->second.c_str(), nullptr);
    };
    int skippedDepth = 0; // inside <defs>, <clipPath>, <mask>, <symbol>: not drawn
    while (nextTag(text, i, e)) {
        const bool container = e.name == "g" || e.name == "a" || e.name == "svg" || e.name == "switch";
        const bool hidden = e.name == "defs" || e.name == "clipPath" || e.name == "mask" || e.name == "symbol" || e.name == "marker" || e.name == "pattern";
        if (e.closing) {
            if (hidden && skippedDepth > 0) --skippedDepth;
            else if (container && stack.size() > 1) stack.pop_back();
            continue;
        }
        if (hidden) {
            if (!e.selfClosing) ++skippedDepth;
            continue;
        }
        if (skippedDepth > 0) continue;
        const Affine local = e.attrs.count("transform") ? parseTransform(e.attrs["transform"]) : Affine{};
        const Affine m = local.then(stack.back());
        if (container) {
            if (!e.selfClosing) stack.push_back(m);
            continue;
        }
        if (e.attrs["display"] == "none" || e.attrs["visibility"] == "hidden") continue;
        const Affine full = m.then(toMm);
        auto T = [&](double x, double y) { return full.apply(x, y); };
        const bool filled = e.attrs.count("fill") ? e.attrs["fill"] != "none" : true;
        if (e.name == "path") {
            svgPath(e.attrs["d"], T, pb);
            pb.paint(false);
        } else if (e.name == "line") {
            pb.moveTo(T(num(e.attrs, "x1"), num(e.attrs, "y1")));
            pb.lineTo(T(num(e.attrs, "x2"), num(e.attrs, "y2")));
            pb.paint(false);
        } else if (e.name == "polyline" || e.name == "polygon") {
            NumReader r(e.attrs["points"]);
            double x, y;
            bool first = true;
            while (r.number(x) && r.number(y)) {
                if (first) pb.moveTo(T(x, y)), first = false;
                else pb.lineTo(T(x, y));
            }
            if (e.name == "polygon") pb.close();
            pb.paint(false);
        } else if (e.name == "rect") {
            const double x = num(e.attrs, "x"), y = num(e.attrs, "y"), rw = num(e.attrs, "width"), rh = num(e.attrs, "height");
            if (rw > 0 && rh > 0) {
                pb.moveTo(T(x, y));
                pb.lineTo(T(x + rw, y));
                pb.lineTo(T(x + rw, y + rh));
                pb.lineTo(T(x, y + rh));
                pb.close();
                pb.paint(false);
            }
        } else if (e.name == "circle" || e.name == "ellipse") {
            const double cx = num(e.attrs, "cx"), cy = num(e.attrs, "cy");
            const double rx = e.name == "circle" ? num(e.attrs, "r") : num(e.attrs, "rx");
            const double ry = e.name == "circle" ? rx : num(e.attrs, "ry");
            if (rx <= 0 || ry <= 0) continue;
            if (std::fabs(rx - ry) < 1e-9 && full.similar()) {
                VectorPath c;
                c.circle = true;
                c.closed = true;
                c.pts = {T(cx, cy)};
                c.radius = static_cast<float>(rx * full.scale());
                pb.out.push_back(c);
            } else {
                const int n = 96;
                for (int s = 0; s <= n; ++s) {
                    const double t = 2 * kPiD * s / n;
                    const Vec2 p = T(cx + rx * std::cos(t), cy + ry * std::sin(t));
                    if (s == 0) pb.moveTo(p);
                    else if (s < n) pb.lineTo(p);
                }
                pb.close();
                pb.paint(false);
            }
        }
        (void)filled;
    }
    std::vector<VectorPath> out = unique(std::move(pb.out));
    if (out.empty() && error) *error = "No line work in the SVG";
    return out;
}

std::vector<VectorPath> readPdfPaths(const std::string& path, int page, double scale, VectorImportReport* report, std::string* error) {
    std::string bytes;
    if (!readFile(path, bytes, error)) return {};
    return parsePdf(bytes, page, scale, report, error);
}

std::vector<VectorPath> readSvgPaths(const std::string& path, double scale, VectorImportReport* report, std::string* error) {
    std::string text;
    if (!readFile(path, text, error)) return {};
    return parseSvg(text, scale, report, error);
}

int addVectorPaths(Scene& scene, SketchPlane plane, const PlaneFrame& frame, const std::vector<VectorPath>& paths) {
    int added = 0;
    for (const VectorPath& p : paths) {
        SketchCurve c;
        c.plane = plane;
        c.frame = frame;
        if (p.circle) {
            c.kind = CurveKind::Circle;
            c.pts = {p.pts[0]};
            c.radius = p.radius;
        } else if (p.pts.size() == 2 && !p.closed) {
            c.kind = CurveKind::Line;
            c.pts = p.pts;
        } else {
            c.kind = CurveKind::Polyline;
            c.pts = p.pts;
            if (p.closed) c.pts.push_back(p.pts.front()); // a closed polyline ends where it began
        }
        scene.sketch().add(std::move(c));
        ++added;
    }
    return added;
}

} // namespace sicdm

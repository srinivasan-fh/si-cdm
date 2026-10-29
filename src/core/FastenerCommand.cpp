// FASTENER: standard bolts, screws, nuts, washers and pins, one at a time or into the
// hole wizard's holes.

#include "core/FastenerCommand.hpp"

#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/Fastener.hpp"
#include "core/Library.hpp"
#include "core/Units.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <map>
#include <optional>

namespace sicdm {

namespace {

std::string lowerOf(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string num(double v, int digits = 4) {
    char b[32];
    std::snprintf(b, sizeof b, "%.*g", digits, std::fabs(v) < 5e-10 ? 0.0 : v);
    return b;
}

int bodyIndex(const Document& d, const std::string& n) {
    const auto& b = d.scene.bodies();
    for (std::size_t i = 0; i < b.size(); ++i)
        if (b[i].name == n) return static_cast<int>(i);
    for (std::size_t i = 0; i < b.size(); ++i)
        if (lowerOf(b[i].name) == lowerOf(n)) return static_cast<int>(i);
    return -1;
}

bool nameTaken(const Document& d, const std::vector<Body>& adding, const std::string& n) {
    auto same = [&](const Body& b) { return lowerOf(b.name) == lowerOf(n) || lowerOf(b.component) == lowerOf(n); };
    return std::any_of(d.scene.bodies().begin(), d.scene.bodies().end(), same) || std::any_of(adding.begin(), adding.end(), same);
}

std::string freeName(const Document& d, const std::vector<Body>& adding, const std::string& base) {
    for (int k = 1;; ++k)
        if (!nameTaken(d, adding, base + " " + std::to_string(k))) return base + " " + std::to_string(k);
}

// A type by its library key or any alias (SHCS, BOLT, NUT, DOWEL...).
std::optional<FastenerType> typeOf(const std::string& word) {
    const LibraryItem* it = findLibraryItem(word);
    if (!it) return std::nullopt;
    for (int t = 0; t < static_cast<int>(FastenerType::Count); ++t)
        if (std::string(it->key) == fastenerKey(static_cast<FastenerType>(t))) return static_cast<FastenerType>(t);
    return std::nullopt;
}

const char* itemName(FastenerType t) {
    const LibraryItem* it = findLibraryItem(fastenerKey(t));
    return it ? it->name : fastenerTypeName(t);
}

// The short word a type is typed as: its first alias.
std::string typeWord(FastenerType t) {
    const LibraryItem* it = findLibraryItem(fastenerKey(t));
    if (!it) return fastenerKey(t);
    const std::string a = it->aliases;
    return a.substr(0, a.find(' '));
}

std::optional<Vec3> directionOf(const std::string& w) {
    const std::string k = lowerOf(w);
    const float sign = !k.empty() && k[0] == '-' ? -1.0f : 1.0f;
    const std::string axis = !k.empty() && (k[0] == '-' || k[0] == '+') ? k.substr(1) : k;
    if (axis == "x") return Vec3{sign, 0, 0};
    if (axis == "y") return Vec3{0, sign, 0};
    if (axis == "z") return Vec3{0, 0, sign};
    const auto v = parseVec3(w);
    if (!v || !(length(*v) > 1e-9f)) return std::nullopt;
    return normalize(*v);
}

// A size, or a size with its length ("M6x20", "1/4-20x3/4", "6x20" for a pin).
bool sizeAndLength(FastenerType type, const std::string& word, int& size, std::optional<float>& len) {
    size = findFastenerSize(type, word);
    if (size >= 0) return true;
    const std::size_t x = lowerOf(word).rfind('x');
    if (x == std::string::npos || x == 0 || x + 1 >= word.size()) return false;
    size = findFastenerSize(type, word.substr(0, x));
    if (size < 0) return false;
    len = parseFastenerLength(word.substr(x + 1), fastenerSizes(type)[static_cast<std::size_t>(size)].inch);
    return len.has_value();
}

// Through a body along `d` from its surface at `p`, beside a hole of radius `clear`: the
// thickness of the material there (the longest of a few tries round the hole; 0 when none
// finds material).
float thicknessAt(const Body& b, const Vec3& p, const Vec3& d, float clear) {
    const AABB box = b.bounds();
    const float size = box.valid() ? length(box.max - box.min) : 100.0f;
    const float eps = 1e-3f * size + 0.01f;
    const Vec3 ref = std::fabs(d.z) < 0.9f ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
    const Vec3 u = normalize(cross(d, ref)), v = cross(d, u);
    const auto& V = b.mesh.vertices;
    const auto& I = b.mesh.indices;
    float best = 0;
    for (int k = 0; k < 8; ++k) {
        const float a = 2.0f * kPi * static_cast<float>(k) / 8.0f;
        const Vec3 o = p + (u * std::cos(a) + v * std::sin(a)) * (clear + std::max(0.5f, 0.15f * clear)) - d * eps;
        std::vector<float> hits;
        for (std::size_t t = 0; t + 2 < I.size(); t += 3) { // Moller-Trumbore, both faces
            const Vec3 p0 = V[I[t]].position + b.offset, p1 = V[I[t + 1]].position + b.offset, p2 = V[I[t + 2]].position + b.offset;
            const Vec3 e1 = p1 - p0, e2 = p2 - p0, h = cross(d, e2);
            const float det = dot(e1, h);
            if (std::fabs(det) < 1e-12f) continue;
            const float f = 1.0f / det;
            const Vec3 s = o - p0;
            const float uu = f * dot(s, h);
            if (uu < 0.0f || uu > 1.0f) continue;
            const Vec3 q = cross(s, e1);
            const float vv = f * dot(d, q);
            if (vv < 0.0f || uu + vv > 1.0f) continue;
            const float tt = f * dot(e2, q);
            if (tt > 0.0f) hits.push_back(tt);
        }
        std::sort(hits.begin(), hits.end());
        // Into the material at the surface, out on the far side (an edge between two
        // triangles can count twice: drop repeats).
        hits.erase(std::unique(hits.begin(), hits.end(), [&](float x, float y) { return y - x < 1e-4f * size; }), hits.end());
        if (hits.size() < 2 || std::fabs(hits[0] - eps) > 2.0f * eps) continue;
        best = std::max(best, hits[1] - hits[0]);
    }
    return best;
}

struct Placed {
    Body body;
    std::string designation;
};

Placed makeFastener(FastenerType type, int size, float len, const Vec3& at, const Vec3& dir, const std::string& name) {
    std::vector<float> params{static_cast<float>(type), static_cast<float>(size)};
    if (fastenerHasLength(type)) params.push_back(len);
    Body b = makeBody(name, "fastener", params, at, fastenerColour(type));
    b.component = name;
    const Vec3 rot = fastenerRotation(dir);
    if (rot.x != 0.0f || rot.y != 0.0f || rot.z != 0.0f) b.setRotation(rot);
    return {std::move(b), fastenerDesignation(params)};
}

std::string sizesRange(FastenerType t) {
    const auto& L = fastenerSizes(t);
    std::string metric, inch;
    for (const FastenerSize& s : L) {
        std::string& r = s.inch ? inch : metric;
        if (r.empty()) r = std::string(fastenerStandard(t, s)) + " " + s.name;
    }
    for (auto it = L.rbegin(); it != L.rend(); ++it) {
        std::string& r = it->inch ? inch : metric;
        if (r.find(" to ") == std::string::npos && r.substr(r.rfind(' ') + 1) != it->name) r += " to " + it->name;
    }
    return metric + (inch.empty() ? "" : "; " + inch);
}

} // namespace

CommandOutput runFastenerCommand(Document& doc, const std::vector<std::string>& a, const std::vector<bool>& quoted) {
    CommandOutput out;
    const char* usage = "FASTENER [LIST] | SIZES type | type size[xlength] [LENGTH l] [AT x,y,z] [DIR x,y,z|-Z...] [NAME n] | "
                        "INTO [body] [HOLE n,...] [type] [SIZE s] [LENGTH l] [GRIP g] [WASHER] [NUT]";
    auto unq = [&](std::size_t i) { return i < quoted.size() && quoted[i]; };
    const std::string sub = a.empty() ? std::string() : lowerOf(a[0]);

    if (a.empty() || (sub == "list" && !unq(0))) {
        out.say("Standard fasteners (FASTENER type size[xlength] [AT x,y,z] [DIR x,y,z]; FASTENER INTO body fills its holes):");
        for (int t = 0; t < static_cast<int>(FastenerType::Count); ++t) {
            const auto type = static_cast<FastenerType>(t);
            out.say("  " + typeWord(type) + "  " + fastenerTypeName(type) + ": " + sizesRange(type));
        }
        out.say("Threads are cosmetic (the shank is modelled at the nominal diameter); each part carries its designation for the parts list.");
        return out;
    }

    if (sub == "sizes" && !unq(0)) {
        if (a.size() < 2) return out.fail("Usage: FASTENER SIZES type (FASTENER lists the types)");
        const auto type = typeOf(a[1]);
        if (!type) return out.fail("No fastener type " + a[1] + " (FASTENER lists them)");
        out.say(std::string(fastenerTypeName(*type)) + " sizes (mm):");
        const auto& L = fastenerSizes(*type);
        for (std::size_t i = 0; i < L.size(); ++i) {
            const FastenerSize& s = L[i];
            std::string line = "  " + s.name + "  " + fastenerStandard(*type, s);
            switch (*type) {
            case FastenerType::HexBolt: line += "  across flats " + num(s.head) + ", head " + num(s.height); break;
            case FastenerType::SocketCap:
            case FastenerType::ButtonHead: line += "  head " + num(s.head) + " x " + num(s.height) + ", socket " + num(s.socket); break;
            case FastenerType::Countersunk: line += "  head " + num(s.head) + ", socket " + num(s.socket); break;
            case FastenerType::SetScrew: line += "  socket " + num(s.socket); break;
            case FastenerType::HexNut: line += "  across flats " + num(s.head) + ", height " + num(s.height); break;
            case FastenerType::Washer: line += "  " + num(s.hole) + " x " + num(s.head) + " x " + num(s.height); break;
            default: line += "  diameter " + num(s.d); break;
            }
            if (fastenerHasLength(*type)) line += ", length from " + fastenerLengthText(minFastenerLength(*type, static_cast<int>(i)), s.inch) + (s.inch ? " in" : "");
            out.say(line);
        }
        return out;
    }

    if (sub == "into" && !unq(0)) {
        // ---- Into the hole wizard's holes ----------------------------------------------------
        int bi = -1;
        std::optional<FastenerType> type;
        std::vector<int> holeIds;
        std::string sizeName, lengthWord;
        float grip = 0;
        bool washer = false, nut = false;
        for (std::size_t i = 1; i < a.size(); ++i) {
            const std::string k = lowerOf(a[i]);
            const bool key = !unq(i);
            auto value = [&]() -> const std::string* { return i + 1 < a.size() ? &a[++i] : nullptr; };
            if (key && (k == "hole" || k == "holes")) {
                const std::string* v = value();
                if (!v) return out.fail("HOLE needs the hole numbers, 1,2,...");
                std::string w = *v;
                for (std::size_t at = 0; at <= w.size();) {
                    const std::size_t comma = std::min(w.find(',', at), w.size());
                    const auto n = parseNumber(w.substr(at, comma - at));
                    if (!n || *n < 1 || std::fabs(*n - std::round(*n)) > 1e-6f) return out.fail("HOLE needs the hole numbers, 1,2,...");
                    holeIds.push_back(static_cast<int>(std::lround(*n)) - 1);
                    at = comma + 1;
                }
            } else if (key && k == "size") {
                const std::string* v = value();
                if (!v) return out.fail("SIZE needs a size (M6, 1/4-20...)");
                sizeName = *v;
            } else if (key && k == "length") {
                const std::string* v = value();
                if (!v) return out.fail("LENGTH needs a length");
                lengthWord = *v;
            } else if (key && k == "grip") {
                const std::string* v = value();
                const auto g = v ? parseLength(*v) : std::nullopt;
                if (!g || *g < 0) return out.fail("GRIP is the thickness clamped under the head");
                grip = static_cast<float>(*g);
            } else if (key && (k == "washer" || k == "washers")) {
                washer = true;
            } else if (key && (k == "nut" || k == "nuts")) {
                nut = true;
            } else if (bi < 0 && bodyIndex(doc, a[i]) >= 0) {
                bi = bodyIndex(doc, a[i]);
            } else if (key && !type && typeOf(a[i])) {
                type = typeOf(a[i]);
            } else {
                return out.fail("No body or fastener type named " + a[i] + " (" + usage + ")");
            }
        }
        if (bi < 0) bi = doc.selectedBody;
        auto& bodies = doc.scene.bodies();
        if (bi < 0 || bi >= static_cast<int>(bodies.size())) return out.fail("Which part? Name it, or select it");
        const Body& part = bodies[static_cast<std::size_t>(bi)];
        if (part.holes.empty()) return out.fail(part.name + " has no holes made with the hole wizard (HOLE ... SIZE M6)");
        if (type && (*type == FastenerType::HexNut || *type == FastenerType::Washer))
            return out.fail("INTO places screws, bolts and pins; WASHER and NUT add washers and nuts to them");
        if (holeIds.empty())
            for (std::size_t h = 0; h < part.holes.size(); ++h) holeIds.push_back(static_cast<int>(h));
        for (int h : holeIds)
            if (h >= static_cast<int>(part.holes.size()))
                return out.fail(part.name + " has " + std::to_string(part.holes.size()) + (part.holes.size() == 1 ? " hole" : " holes"));

        std::vector<Body> adding;
        std::vector<std::string> lines;
        std::map<std::string, int> count;
        for (int h : holeIds) {
            const HoleNote& note = part.holes[static_cast<std::size_t>(h)];
            const std::string which = "Hole " + std::to_string(h + 1);
            const Vec3 hp = note.at + part.offset;
            if (!(length(note.dir) > 0)) return out.fail(which + " has no direction");
            const Vec3 d = normalize(note.dir);
            const bool tapped = note.threadDia > 0;
            const FastenerType ft = type ? *type
                                    : note.csinkDia > 0 ? FastenerType::Countersunk
                                    : note.cboreDia > 0 || tapped ? FastenerType::SocketCap
                                                                  : FastenerType::HexBolt;
            const std::string sz = !sizeName.empty() ? sizeName : note.size;
            if (sz.empty()) return out.fail(which + " has no standard size (made by DIAMETER): give SIZE");
            const int idx = findFastenerSize(ft, sz);
            if (idx < 0)
                return out.fail(std::string(fastenerTypeName(ft)) + "s don't come in " + sz + " (FASTENER SIZES " + typeWord(ft) + " lists them)");
            const FastenerSize& z = fastenerSizes(ft)[static_cast<std::size_t>(idx)];
            const bool pin = ft == FastenerType::DowelPin, set = ft == FastenerType::SetScrew, csk = ft == FastenerType::Countersunk;
            // The material along the hole: its depth, or measured through the part.
            float t = note.depth;
            const bool through = !(note.depth > 0);
            if (through) {
                t = thicknessAt(part, hp, d, 0.5f * std::max({note.diameter, note.cboreDia, note.csinkDia, z.d}));
                if (!(t > 0)) return out.fail(which + ": can't find the part's thickness along it");
            }
            if (nut && !through) return out.fail(which + " is blind: NUT needs a hole through");
            if (nut && (pin || set)) return out.fail("NUT goes with bolts and screws");
            // Along d from the surface (negative: outside it): where the fastener's origin sits.
            const float seat = csk ? 0.0f : note.cboreDia > 0 && !pin && !set ? note.cboreDepth : -grip;
            // (No washer under a head sunk in a counterbore: it wouldn't fit.)
            const bool headWasher = washer && !csk && !pin && !set && !(note.cboreDia > 0);
            const int wi = washer ? findFastenerSize(FastenerType::Washer, z.name) : -1;
            if ((headWasher || (washer && nut)) && wi < 0) return out.fail("No plain washer for " + z.name);
            const float hw = wi >= 0 ? fastenerSizes(FastenerType::Washer)[static_cast<std::size_t>(wi)].height : 0.0f;
            const int ni = nut ? findFastenerSize(FastenerType::HexNut, z.name) : -1;
            if (nut && ni < 0) return out.fail("No hex nut for " + z.name);
            const float m = ni >= 0 ? fastenerSizes(FastenerType::HexNut)[static_cast<std::size_t>(ni)].height : 0.0f;
            float origin = pin ? 0.0f : seat - (headWasher ? hw : 0.0f);
            float len = 0;
            if (!lengthWord.empty()) {
                const auto l = parseFastenerLength(lengthWord, z.inch);
                if (!l) return out.fail("LENGTH: " + lengthWord + " is not a length");
                len = *l;
            } else if (pin) {
                len = pickFastenerLength(ft, idx, 2.0f * t, true); // half in the hole
            } else if (set) {
                len = pickFastenerLength(ft, idx, tapped && note.threadDepth > 0 ? note.threadDepth : t, true);
            } else {
                // Down to the tip: through the part and the nut (two threads past it), into a
                // tapped thread (1.5 d, no deeper than the thread), or 1.5 d into what's behind.
                const float thread = note.threadDepth > 0 ? note.threadDepth : t;
                float tip = 0;
                bool atMost = false;
                if (nut) tip = t + (washer ? hw : 0.0f) + m + 2.0f * z.pitch;
                else if (tapped) tip = std::min(thread, 1.5f * z.d);
                else if (through) tip = t + 1.5f * z.d;
                else tip = t, atMost = true; // a plain blind hole: no deeper than it
                len = pickFastenerLength(ft, idx, tip - origin, atMost);
                if (tapped && !through && origin + len > thread + 1e-3f) // it would bottom out: the longest that doesn't
                    len = pickFastenerLength(ft, idx, thread - origin, true);
            }
            if (len < minFastenerLength(ft, idx) - 1e-4f)
                return out.fail(which + ": " + std::string(fastenerTypeName(ft)) + "s " + z.name + " are at least " +
                                fastenerLengthText(minFastenerLength(ft, idx), z.inch) + (z.inch ? " in" : " mm") + " long");
            std::string line = "  " + which + ": ";
            Placed main = makeFastener(ft, idx, len, hp + d * origin, d, freeName(doc, adding, itemName(ft)));
            if (main.body.mesh.indices.empty()) return out.fail(which + ": that fastener doesn't make a sound part");
            line += main.designation;
            ++count[main.designation];
            adding.push_back(std::move(main.body));
            if (headWasher) {
                Placed w = makeFastener(FastenerType::Washer, wi, 0, hp + d * (seat - hw), d, freeName(doc, adding, itemName(FastenerType::Washer)));
                line += " + " + w.designation;
                ++count[w.designation];
                adding.push_back(std::move(w.body));
            }
            if (nut) {
                if (washer) {
                    Placed w = makeFastener(FastenerType::Washer, wi, 0, hp + d * t, d, freeName(doc, adding, itemName(FastenerType::Washer)));
                    line += " + " + w.designation;
                    ++count[w.designation];
                    adding.push_back(std::move(w.body));
                }
                Placed n = makeFastener(FastenerType::HexNut, ni, 0, hp + d * (t + (washer ? hw : 0.0f)), d, freeName(doc, adding, itemName(FastenerType::HexNut)));
                line += " + " + n.designation;
                ++count[n.designation];
                adding.push_back(std::move(n.body));
            }
            if (washer && !headWasher && note.cboreDia > 0 && !csk && !pin && !set) line += "  (no washer under a head in a counterbore)";
            if (through) line += "  (through " + lengthText(t) + ")";
            lines.push_back(line);
        }
        const std::string partName = part.name;
        for (Body& b : adding) doc.scene.add(std::move(b));
        doc.dirty = true;
        out.say("Fastened " + std::to_string(holeIds.size()) + (holeIds.size() == 1 ? " hole" : " holes") + " in " + partName + ":");
        for (const std::string& l : lines) out.say(l);
        std::string parts;
        for (const auto& [des, n] : count) parts += (parts.empty() ? "" : ", ") + std::to_string(n) + " x " + des;
        out.say("Parts: " + parts);
        return out;
    }

    // ---- One fastener ----------------------------------------------------------------------
    const auto type = typeOf(a[0]);
    if (!type) return out.fail("No fastener type " + a[0] + " (FASTENER lists them; " + usage + ")");
    if (a.size() < 2) return out.fail("Which size? FASTENER SIZES " + typeWord(*type) + " lists them");
    int idx = -1;
    std::optional<float> len;
    if (!sizeAndLength(*type, a[1], idx, len))
        return out.fail(std::string(fastenerTypeName(*type)) + "s don't come in " + a[1] + " (FASTENER SIZES " + typeWord(*type) + " lists them)");
    const FastenerSize& z = fastenerSizes(*type)[static_cast<std::size_t>(idx)];
    std::optional<Vec3> at;
    Vec3 dir{0, 0, -1};
    std::string name;
    for (std::size_t i = 2; i < a.size(); ++i) {
        const std::string k = lowerOf(a[i]);
        const bool key = !unq(i);
        if (key && !a[i].empty() && a[i][0] == '@') {
            if (!(at = parseVec3(a[i].substr(1)))) return out.fail("Position needs @x,y,z");
        } else if (key && (k == "at" || k == "position") && i + 1 < a.size()) {
            if (!(at = parseVec3(a[++i]))) return out.fail("AT needs x,y,z");
        } else if (key && k == "dir" && i + 1 < a.size()) {
            const auto v = directionOf(a[++i]);
            if (!v) return out.fail("DIR is X, Y, Z (-X...) or a direction x,y,z: the way the shank points");
            dir = *v;
        } else if (key && k == "length" && i + 1 < a.size()) {
            if (!(len = parseFastenerLength(a[++i], z.inch))) return out.fail("LENGTH: " + a[i] + " is not a length");
        } else if (key && k == "name" && i + 1 < a.size()) {
            name = a[++i];
        } else {
            return out.fail("Unknown option '" + a[i] + "' (" + usage + ")");
        }
    }
    if (!fastenerHasLength(*type)) len.reset();
    else if (!len) len = pickFastenerLength(*type, idx, 3.0f * z.d);
    if (len && *len < minFastenerLength(*type, idx) - 1e-4f)
        return out.fail(std::string(fastenerTypeName(*type)) + "s " + z.name + " are at least " + fastenerLengthText(minFastenerLength(*type, idx), z.inch) +
                        (z.inch ? " in" : " mm") + " long");
    std::vector<Body> none;
    if (name.empty()) name = freeName(doc, none, itemName(*type));
    else if (nameTaken(doc, none, name)) return out.fail("A body or component named " + name + " already exists");
    Placed p = makeFastener(*type, idx, len.value_or(0.0f), at.value_or(Vec3{}), dir, name);
    if (p.body.mesh.indices.empty()) return out.fail("That fastener doesn't make a sound part");
    if (!at && !doc.scene.bodies().empty()) { // beside the model, like PART
        AABB scene;
        for (const Body& b : doc.scene.bodies()) scene.expand(b.bounds());
        const AABB own = p.body.bounds();
        if (scene.valid() && own.valid()) p.body.offset += Vec3{scene.max.x + 30.0f - own.min.x, -own.center().y, 0.0f};
    }
    const std::string said = "Inserted " + name + ": " + p.designation + " (" + fastenerDescription(p.body.params) + ")";
    doc.scene.add(std::move(p.body));
    doc.selectedBody = static_cast<int>(doc.scene.bodies().size()) - 1;
    doc.selectedCurve = doc.selectedImage = -1;
    doc.dirty = true;
    out.say(said);
    return out;
}

} // namespace sicdm

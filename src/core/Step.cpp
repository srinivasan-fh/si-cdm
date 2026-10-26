#include "core/Step.hpp"

#include "core/Extrude.hpp"

#include "core/Scene.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <filesystem>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>

namespace sicdm {

namespace {

// --- Part 21 values -----------------------------------------------------------------

struct Value {
    enum Type : unsigned char { Null, Star, Num, Str, Enum, Ref, List, Typed } type = Null;
    double num = 0;
    long long ref = 0;
    std::string s;          // string, enum (without dots), typed parameter's type
    std::vector<Value> list; // list items, typed parameter's arguments
    bool isRef() const { return type == Ref; }
};

struct Record {
    std::string type;
    std::vector<Value> args;
};

struct Entity {
    std::vector<Record> parts; // one for a simple entity; several (sorted by name) for a complex one
    const Record* part(const char* name) const {
        for (const Record& r : parts)
            if (r.type == name) return &r;
        return nullptr;
    }
    const std::string& type() const {
        static const std::string empty;
        return parts.size() == 1 ? parts[0].type : empty;
    }
};

class Parser {
public:
    explicit Parser(const std::string& t) : s(t), n(t.size()) {}

    bool parse(std::unordered_map<long long, Entity>& out, std::string& schema, std::string& err) {
        // Header: the schema name.
        const std::size_t fs = s.find("FILE_SCHEMA");
        if (fs != std::string::npos) {
            const std::size_t q = s.find('\'', fs);
            const std::size_t e = q == std::string::npos ? q : s.find('\'', q + 1);
            if (e != std::string::npos) schema = s.substr(q + 1, e - q - 1);
        }
        std::size_t data = s.find("DATA;");
        if (data == std::string::npos) {
            err = "not a STEP file (no DATA section)";
            return false;
        }
        i = data + 5;
        for (;;) {
            skip();
            if (i >= n) break;
            if (s.compare(i, 7, "ENDSEC;") == 0) break;
            if (s[i] != '#') {
                // Skip to the next statement.
                while (i < n && s[i] != ';') ++i;
                ++i;
                continue;
            }
            ++i;
            const long long id = integer();
            skip();
            if (i >= n || s[i] != '=') return fail(err, "expected '=' after #" + std::to_string(id));
            ++i;
            skip();
            Entity e;
            if (i < n && s[i] == '(') { // a complex entity: ( A(...) B(...) ... )
                ++i;
                for (;;) {
                    skip();
                    if (i < n && s[i] == ')') {
                        ++i;
                        break;
                    }
                    Record r;
                    r.type = keyword();
                    if (r.type.empty()) return fail(err, "bad complex entity #" + std::to_string(id));
                    skip();
                    if (!args(r.args)) return fail(err, "bad parameters in #" + std::to_string(id));
                    e.parts.push_back(std::move(r));
                }
            } else {
                Record r;
                r.type = keyword();
                skip();
                if (r.type.empty() || !args(r.args)) return fail(err, "bad entity #" + std::to_string(id));
                e.parts.push_back(std::move(r));
            }
            skip();
            if (i < n && s[i] == ';') ++i;
            out[id] = std::move(e);
        }
        return true;
    }

private:
    const std::string& s;
    std::size_t n, i = 0;

    static bool fail(std::string& err, const std::string& m) {
        err = m;
        return false;
    }
    void skip() {
        for (;;) {
            while (i < n && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
            if (i + 1 < n && s[i] == '/' && s[i + 1] == '*') {
                const std::size_t e = s.find("*/", i + 2);
                i = e == std::string::npos ? n : e + 2;
                continue;
            }
            break;
        }
    }
    long long integer() {
        long long v = 0;
        bool neg = false;
        if (i < n && (s[i] == '-' || s[i] == '+')) neg = s[i++] == '-';
        while (i < n && std::isdigit(static_cast<unsigned char>(s[i]))) v = v * 10 + (s[i++] - '0');
        return neg ? -v : v;
    }
    std::string keyword() {
        const std::size_t b = i;
        while (i < n && (std::isalnum(static_cast<unsigned char>(s[i])) || s[i] == '_' || s[i] == '!')) ++i;
        return s.substr(b, i - b);
    }
    bool args(std::vector<Value>& out) {
        if (i >= n || s[i] != '(') return false;
        ++i;
        for (;;) {
            skip();
            if (i >= n) return false;
            if (s[i] == ')') {
                ++i;
                return true;
            }
            Value v;
            if (!value(v)) return false;
            out.push_back(std::move(v));
            skip();
            if (i < n && s[i] == ',') ++i;
        }
    }
    bool value(Value& v) {
        skip();
        if (i >= n) return false;
        const char c = s[i];
        if (c == '$') {
            ++i;
            v.type = Value::Null;
        } else if (c == '*') {
            ++i;
            v.type = Value::Star;
        } else if (c == '#') {
            ++i;
            v.type = Value::Ref;
            v.ref = integer();
        } else if (c == '\'') {
            ++i;
            v.type = Value::Str;
            while (i < n) {
                if (s[i] == '\'') {
                    if (i + 1 < n && s[i + 1] == '\'') {
                        v.s += '\'';
                        i += 2;
                        continue;
                    }
                    ++i;
                    break;
                }
                v.s += s[i++];
            }
        } else if (c == '"') { // binary
            ++i;
            v.type = Value::Str;
            while (i < n && s[i] != '"') v.s += s[i++];
            ++i;
        } else if (c == '.') {
            ++i;
            v.type = Value::Enum;
            while (i < n && s[i] != '.') v.s += s[i++];
            ++i;
        } else if (c == '(') {
            v.type = Value::List;
            return args(v.list);
        } else if (std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+') {
            v.type = Value::Num;
            const char* b = s.c_str() + i;
            char* e = nullptr;
            v.num = std::strtod(b, &e);
            i += static_cast<std::size_t>(e - b);
        } else if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
            v.type = Value::Typed;
            v.s = keyword();
            skip();
            return args(v.list);
        } else {
            return false;
        }
        return true;
    }
};

// --- from entities to the B-rep --------------------------------------------------------

class Reader {
public:
    Reader(const std::unordered_map<long long, Entity>& e, StepReadReport& rep) : m_e(e), m_rep(rep) {}

    std::vector<StepPart> run(std::string& err) {
        findUnits();
        // Solid and surface items, by the representation that holds them.
        std::vector<std::pair<long long, long long>> items; // (representation, item)
        std::set<long long> inRep;
        for (const auto& [id, ent] : m_e) {
            if (ent.parts.size() != 1) continue;
            const std::string& t = ent.type();
            if (t.find("REPRESENTATION") == std::string::npos || t.find("RELATIONSHIP") != std::string::npos) continue;
            const Record& r = ent.parts[0];
            if (r.args.size() < 2 || r.args[1].type != Value::List) continue;
            for (const Value& v : r.args[1].list)
                if (v.isRef() && isSolid(v.ref)) {
                    items.push_back({id, v.ref});
                    inRep.insert(v.ref);
                }
        }
        // Solids no representation names (loose in the file) still come in.
        for (const auto& [id, ent] : m_e)
            if (isSolid(id) && !inRep.count(id) && !usedByOther(id)) items.push_back({-1, id});
        buildPlacements();
        // A solid listed by several representations (a part's own shape and the one its
        // product or an assembly refers to) is one part: read it once, placed wherever any of
        // them is used.
        std::map<long long, std::vector<long long>> repsOf; // item -> its representations
        for (const auto& [rep, item] : items) repsOf[item].push_back(rep);
        std::vector<std::pair<long long, long long>> unique;
        for (auto& [item, reps] : repsOf) {
            std::sort(reps.begin(), reps.end());
            unique.push_back({reps.front(), item});
        }
        std::sort(unique.begin(), unique.end());
        std::vector<StepPart> parts;
        bool firstUnit = true;
        for (const auto& [firstRep, item] : unique) {
            const std::vector<long long>& reps = repsOf[item];
            // The representation that says most: one placed in an assembly, else one with a product.
            long long rep = firstRep;
            for (long long r : reps)
                if (r >= 0 && m_parents.count(classOf(r))) {
                    rep = r;
                    break;
                }
            m_scale = rep >= 0 ? repScale(rep) : m_rep.unitToMm;
            if (firstUnit) m_rep.unitToMm = m_scale, firstUnit = false; // what the parts are in
            m_angle = rep >= 0 ? repAngle(rep) : m_angleDefault;
            auto brep = std::make_shared<Brep>();
            m_b = brep.get();
            m_vertexIdx.clear(), m_edgeIdx.clear(), m_curveIdx.clear(), m_surfIdx.clear();
            if (!solid(item)) {
                m_rep.warnings.push_back("#" + std::to_string(item) + " could not be read");
                continue;
            }
            std::string name = itemName(item);
            for (long long r : reps) {
                const std::string pn = r >= 0 ? productName(r) : std::string();
                if (!pn.empty()) {
                    name = pn;
                    break;
                }
            }
            // Every placement of every representation holding it (each class of the same
            // coordinates once); unplaced representations only when none is placed.
            std::vector<Instance> where;
            std::set<long long> classes;
            for (long long r : reps)
                if (r >= 0 && m_parents.count(classOf(r)) && classes.insert(classOf(r)).second)
                    for (const Instance& in : instances(r)) where.push_back(in);
            if (where.empty()) where = rep >= 0 ? instances(rep) : std::vector<Instance>{Instance{}};
            for (std::size_t k = 0; k < where.size(); ++k) {
                StepPart p;
                p.name = name.empty() ? "STEP part" : name;
                p.assembly = where[k].path;
                if (k == 0 && where.size() == 1) {
                    p.brep = brep;
                } else {
                    p.brep = std::make_shared<Brep>(*brep);
                }
                if (!isIdentity(where[k].p)) p.brep->transform(where[k].p);
                p.brep->name = p.name;
                parts.push_back(std::move(p));
            }
        }
        if (parts.empty()) err = "no solids or surface models in the file";
        return parts;
    }

private:
    const std::unordered_map<long long, Entity>& m_e;
    StepReadReport& m_rep;
    Brep* m_b = nullptr;
    double m_scale = 1.0, m_angle = 1.0, m_angleDefault = 1.0;
    std::unordered_map<long long, int> m_vertexIdx, m_edgeIdx, m_curveIdx, m_surfIdx;
    std::unordered_map<long long, double> m_ctxScale, m_ctxAngle;
    // Representation placements: class (plain links merged) -> its parents with transforms.
    std::unordered_map<long long, long long> m_class;
    std::multimap<long long, std::pair<long long, Placement>> m_parents;

    const Entity* get(long long id) const {
        const auto it = m_e.find(id);
        return it == m_e.end() ? nullptr : &it->second;
    }
    const Record* rec(long long id, const char* type = nullptr) const {
        const Entity* e = get(id);
        if (!e) return nullptr;
        if (!type) return e->parts.size() == 1 ? &e->parts[0] : nullptr;
        return e->part(type);
    }
    static double num(const Value& v, double def = 0) {
        if (v.type == Value::Num) return v.num;
        if (v.type == Value::Typed && !v.list.empty()) return num(v.list[0], def);
        return def;
    }
    static bool flag(const Value& v, bool def = true) { return v.type == Value::Enum ? v.s == "T" : def; }
    static bool isIdentity(const Placement& p) {
        return p.s == 1 && p.t.x == 0 && p.t.y == 0 && p.t.z == 0 && p.r[0] == 1 && p.r[4] == 1 && p.r[8] == 1 && p.r[1] == 0 && p.r[2] == 0 && p.r[3] == 0 &&
               p.r[5] == 0 && p.r[6] == 0 && p.r[7] == 0;
    }

    bool isSolid(long long id) const {
        const Entity* e = get(id);
        if (!e || e->parts.size() != 1) return false;
        const std::string& t = e->type();
        return t == "MANIFOLD_SOLID_BREP" || t == "BREP_WITH_VOIDS" || t == "FACETED_BREP" || t == "SHELL_BASED_SURFACE_MODEL";
    }
    bool usedByOther(long long id) const {
        (void)id;
        return false;
    }

    // --- units ---
    double lengthUnit(long long id, int depth = 0) const {
        const Entity* e = get(id);
        if (!e || depth > 5) return 1.0;
        if (const Record* si = e->part("SI_UNIT")) {
            double f = 1000.0; // metres
            if (!si->args.empty() && si->args.size() >= 2 && si->args[si->args.size() - 2].type == Value::Enum) {
                const std::string& p = si->args[si->args.size() - 2].s;
                static const std::map<std::string, double> prefix = {{"MILLI", 1e-3}, {"CENTI", 1e-2}, {"DECI", 1e-1}, {"KILO", 1e3}, {"MICRO", 1e-6},
                                                                      {"NANO", 1e-9}, {"DECA", 10}, {"HECTO", 100}};
                const auto it = prefix.find(p);
                if (it != prefix.end()) f *= it->second;
            }
            return f;
        }
        if (const Record* cb = e->part("CONVERSION_BASED_UNIT")) {
            if (cb->args.size() >= 2 && cb->args[1].isRef()) {
                const Record* m = rec(cb->args[1].ref);
                if (m && m->args.size() >= 2) return num(m->args[0], 1.0) * (m->args[1].isRef() ? lengthUnit(m->args[1].ref, depth + 1) : 1.0);
            }
        }
        return 1.0;
    }
    double angleUnit(long long id, int depth = 0) const {
        const Entity* e = get(id);
        if (!e || depth > 5) return 1.0;
        if (e->part("SI_UNIT")) return 1.0; // radians
        if (const Record* cb = e->part("CONVERSION_BASED_UNIT")) {
            if (cb->args.size() >= 2 && cb->args[1].isRef()) {
                const Record* m = rec(cb->args[1].ref);
                if (m && m->args.size() >= 2) return num(m->args[0], 1.0) * (m->args[1].isRef() ? angleUnit(m->args[1].ref, depth + 1) : 1.0);
            }
        }
        return 1.0;
    }
    void findUnits() {
        bool first = true;
        for (const auto& [id, ent] : m_e) {
            const Record* ga = ent.part("GLOBAL_UNIT_ASSIGNED_CONTEXT");
            if (!ga || ga->args.empty() || ga->args[0].type != Value::List) continue;
            double len = 1.0, ang = 1.0;
            for (const Value& u : ga->args[0].list) {
                if (!u.isRef()) continue;
                const Entity* ue = get(u.ref);
                if (!ue) continue;
                if (ue->part("LENGTH_UNIT")) len = lengthUnit(u.ref);
                else if (ue->part("PLANE_ANGLE_UNIT")) ang = angleUnit(u.ref);
            }
            m_ctxScale[id] = len;
            m_ctxAngle[id] = ang;
            if (first) {
                m_rep.unitToMm = len;
                m_angleDefault = ang;
                first = false;
            }
        }
    }
    double repScale(long long rep) const {
        const Record* r = rec(rep);
        if (r && r->args.size() >= 3 && r->args[2].isRef()) {
            const auto it = m_ctxScale.find(r->args[2].ref);
            if (it != m_ctxScale.end()) return it->second;
        }
        return m_rep.unitToMm;
    }
    double repAngle(long long rep) const {
        const Record* r = rec(rep);
        if (r && r->args.size() >= 3 && r->args[2].isRef()) {
            const auto it = m_ctxAngle.find(r->args[2].ref);
            if (it != m_ctxAngle.end()) return it->second;
        }
        return m_angleDefault;
    }

    // --- names ---
    std::string itemName(long long id) const {
        const Record* r = rec(id);
        return r && !r->args.empty() && r->args[0].type == Value::Str ? r->args[0].s : std::string();
    }
    std::string productName(long long rep) const {
        // SHAPE_DEFINITION_REPRESENTATION(definition, rep) -> PRODUCT_DEFINITION_SHAPE -> PRODUCT_DEFINITION -> FORMATION -> PRODUCT.
        std::set<long long> reps{rep};
        const long long cls = classOf(rep);
        for (const auto& [id, c] : m_class)
            if (c == cls) reps.insert(id);
        for (const auto& [id, ent] : m_e) {
            if (ent.type() != "SHAPE_DEFINITION_REPRESENTATION") continue;
            const Record& r = ent.parts[0];
            if (r.args.size() < 2 || !r.args[1].isRef() || !reps.count(r.args[1].ref) || !r.args[0].isRef()) continue;
            const Record* pds = rec(r.args[0].ref);
            if (!pds || pds->args.size() < 3 || !pds->args[2].isRef()) continue;
            const Record* pd = rec(pds->args[2].ref);
            if (!pd || pd->args.size() < 3 || !pd->args[2].isRef()) continue;
            const Record* pdf = rec(pd->args[2].ref);
            if (!pdf || pdf->args.size() < 3 || !pdf->args[2].isRef()) continue;
            const Record* prod = rec(pdf->args[2].ref);
            if (!prod || prod->args.size() < 2) continue;
            const std::string nm = prod->args[1].type == Value::Str && !prod->args[1].s.empty() ? prod->args[1].s : prod->args[0].s;
            if (!nm.empty()) return nm;
        }
        return {};
    }

    // --- assembly placements ---
    long long classOf(long long rep) const {
        long long c = rep;
        for (int guard = 0; guard < 64; ++guard) {
            const auto it = m_class.find(c);
            if (it == m_class.end() || it->second == c) return c;
            c = it->second;
        }
        return c;
    }
    void unite(long long a, long long b) {
        const long long ca = classOf(a), cb = classOf(b);
        if (ca != cb) m_class[ca] = cb;
        if (!m_class.count(cb)) m_class[cb] = cb;
    }
    bool frame(long long id, Frame& f) const {
        const Record* a = rec(id, "AXIS2_PLACEMENT_3D");
        if (!a) return false;
        DVec3 o{0, 0, 0}, z{0, 0, 1}, x{1, 0, 0};
        if (a->args.size() > 1 && a->args[1].isRef()) o = point(a->args[1].ref);
        if (a->args.size() > 2 && a->args[2].isRef()) z = direction(a->args[2].ref);
        if (a->args.size() > 3 && a->args[3].isRef()) x = direction(a->args[3].ref);
        f = Frame::make(o, z, x);
        return true;
    }
    void buildPlacements() {
        for (const auto& [id, ent] : m_e) {
            const Record* rr = ent.part("REPRESENTATION_RELATIONSHIP");
            if (!rr) {
                if (ent.type() == "SHAPE_REPRESENTATION_RELATIONSHIP" || ent.type() == "REPRESENTATION_RELATIONSHIP") rr = &ent.parts[0];
                else continue;
            }
            if (rr->args.size() < 4 || !rr->args[2].isRef() || !rr->args[3].isRef()) continue;
            const long long rep1 = rr->args[2].ref, rep2 = rr->args[3].ref;
            const Record* tr = ent.part("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION");
            if (!tr) {
                unite(rep1, rep2); // the same coordinates
                continue;
            }
            Placement p;
            if (!tr->args.empty() && tr->args[0].isRef()) {
                const Record* idt = rec(tr->args[0].ref, "ITEM_DEFINED_TRANSFORMATION");
                Frame f1, f2;
                if (idt && idt->args.size() >= 4 && idt->args[2].isRef() && idt->args[3].isRef() && frame(idt->args[2].ref, f1) && frame(idt->args[3].ref, f2))
                    p = Placement::fromFrames(f1, f2);
            }
            // rep_1 (the part) is placed in rep_2 (the assembly).
            m_pendingLinks.push_back({rep1, rep2, p});
        }
        for (const auto& l : m_pendingLinks) m_parents.insert({classOf(l.child), {classOf(l.parent), l.p}});
    }
    struct Link {
        long long child, parent;
        Placement p;
    };
    std::vector<Link> m_pendingLinks;

    // Each place the representation is used: where, and the assemblies it sits in (outermost first).
    struct Instance {
        Placement p;
        std::string path;
    };
    std::vector<Instance> instances(long long rep, int depth = 0) {
        const long long c = classOf(rep);
        const auto range = m_parents.equal_range(c);
        if (range.first == range.second || depth > 32) return {Instance{}};
        std::vector<Instance> out;
        for (auto it = range.first; it != range.second; ++it) {
            std::string name = assemblyName(it->second.first);
            for (char& ch : name)
                if (ch == '/') ch = '-'; // the path separator
            for (const Instance& up : instances(it->second.first, depth + 1))
                out.push_back({it->second.second.then(up.p), name.empty() ? up.path : up.path.empty() ? name : up.path + "/" + name});
        }
        return out;
    }
    std::string assemblyName(long long rep) {
        const auto it = m_assemblyNames.find(rep);
        if (it != m_assemblyNames.end()) return it->second;
        return m_assemblyNames[rep] = productName(rep);
    }
    std::map<long long, std::string> m_assemblyNames;

    // --- geometry ---
    DVec3 point(long long id) const {
        const Record* r = rec(id, "CARTESIAN_POINT");
        if (!r || r->args.size() < 2 || r->args[1].type != Value::List) return {};
        const auto& l = r->args[1].list;
        return DVec3{l.size() > 0 ? num(l[0]) : 0, l.size() > 1 ? num(l[1]) : 0, l.size() > 2 ? num(l[2]) : 0} * m_scale;
    }
    DVec3 direction(long long id) const {
        const Record* r = rec(id, "DIRECTION");
        if (!r || r->args.size() < 2 || r->args[1].type != Value::List) return {0, 0, 1};
        const auto& l = r->args[1].list;
        return dnormalize(DVec3{l.size() > 0 ? num(l[0]) : 0, l.size() > 1 ? num(l[1]) : 0, l.size() > 2 ? num(l[2]) : 0});
    }
    DVec3 vector(long long id) const { // VECTOR(name, direction, magnitude)
        const Record* r = rec(id, "VECTOR");
        if (!r || r->args.size() < 3) return {0, 0, 1};
        return direction(r->args[1].ref) * (num(r->args[2], 1.0) * m_scale);
    }
    Frame placement(const Value& v) const {
        Frame f;
        if (v.isRef()) frame(v.ref, f);
        return f;
    }

    static std::vector<double> knots(const Value& mults, const Value& ks) {
        std::vector<double> out;
        for (std::size_t i = 0; i < mults.list.size() && i < ks.list.size(); ++i)
            for (int k = 0; k < static_cast<int>(num(mults.list[i])); ++k) out.push_back(num(ks.list[i]));
        return out;
    }

    int curve(long long id, int depth = 0) {
        if (const auto it = m_curveIdx.find(id); it != m_curveIdx.end()) return it->second;
        const Entity* e = get(id);
        if (!e || depth > 8) return -1;
        BCurve c;
        bool ok = true;
        const std::string& t = e->type();
        if (t == "SURFACE_CURVE" || t == "SEAM_CURVE" || t == "INTERSECTION_CURVE" || t == "BOUNDED_SURFACE_CURVE") {
            const Record& r = e->parts[0];
            return r.args.size() > 1 && r.args[1].isRef() ? m_curveIdx[id] = curve(r.args[1].ref, depth + 1) : -1;
        }
        if (t == "TRIMMED_CURVE") {
            const Record& r = e->parts[0];
            return r.args.size() > 1 && r.args[1].isRef() ? m_curveIdx[id] = curve(r.args[1].ref, depth + 1) : -1;
        }
        if (t == "LINE") {
            const Record& r = e->parts[0];
            c.kind = BCurveKind::Line;
            c.f.o = r.args.size() > 1 ? point(r.args[1].ref) : DVec3{};
            c.f.x = r.args.size() > 2 ? dnormalize(vector(r.args[2].ref)) : DVec3{1, 0, 0};
        } else if (t == "CIRCLE") {
            const Record& r = e->parts[0];
            c.kind = BCurveKind::Circle;
            c.f = placement(r.args[1]);
            c.r1 = num(r.args[2]) * m_scale;
        } else if (t == "ELLIPSE") {
            const Record& r = e->parts[0];
            c.kind = BCurveKind::Ellipse;
            c.f = placement(r.args[1]);
            c.r1 = num(r.args[2]) * m_scale;
            c.r2 = num(r.args[3]) * m_scale;
        } else if (t == "PARABOLA") { // (name, position, focal_dist)
            const Record& r = e->parts[0];
            if (r.args.size() < 3) return -1;
            c.kind = BCurveKind::Parabola;
            c.f = placement(r.args[1]);
            c.r1 = num(r.args[2]) * m_scale;
        } else if (t == "HYPERBOLA") { // (name, position, semi_axis, semi_imag_axis)
            const Record& r = e->parts[0];
            if (r.args.size() < 4) return -1;
            c.kind = BCurveKind::Hyperbola;
            c.f = placement(r.args[1]);
            c.r1 = num(r.args[2]) * m_scale;
            c.r2 = num(r.args[3]) * m_scale;
        } else if (t == "POLYLINE") {
            const Record& r = e->parts[0];
            c.kind = BCurveKind::Polyline;
            for (const Value& v : r.args[1].list) c.pts.push_back(point(v.ref));
        } else if (t == "B_SPLINE_CURVE_WITH_KNOTS") {
            const Record& r = e->parts[0];
            // (name, degree, points, form, closed, self_intersect, mults, knots, spec)
            if (r.args.size() < 8) return -1;
            c.kind = BCurveKind::BSpline;
            c.bs.degree = static_cast<int>(num(r.args[1]));
            for (const Value& v : r.args[2].list) c.bs.cp.push_back(point(v.ref));
            c.bs.knots = knots(r.args[6], r.args[7]);
        } else if (const Record* b = e->part("B_SPLINE_CURVE")) {
            // Complex: B_SPLINE_CURVE(degree, points, form, closed, si) B_SPLINE_CURVE_WITH_KNOTS(mults, knots, spec)
            // RATIONAL_B_SPLINE_CURVE(weights)
            c.kind = BCurveKind::BSpline;
            if (b->args.size() < 2) return -1;
            c.bs.degree = static_cast<int>(num(b->args[0]));
            for (const Value& v : b->args[1].list) c.bs.cp.push_back(point(v.ref));
            if (const Record* k = e->part("B_SPLINE_CURVE_WITH_KNOTS"); k && k->args.size() >= 2) c.bs.knots = knots(k->args[0], k->args[1]);
            else ok = false;
            if (const Record* w = e->part("RATIONAL_B_SPLINE_CURVE"); w && !w->args.empty())
                for (const Value& v : w->args[0].list) c.bs.w.push_back(num(v));
        } else {
            m_rep.warnings.push_back("curve " + (t.empty() ? std::string("(complex)") : t) + " is not supported");
            return -1;
        }
        if (c.kind == BCurveKind::BSpline &&
            (c.bs.cp.size() < 2 || c.bs.knots.size() != c.bs.cp.size() + static_cast<std::size_t>(c.bs.degree) + 1 || (!c.bs.w.empty() && c.bs.w.size() != c.bs.cp.size())))
            ok = false;
        if (!ok) return -1;
        m_b->curves.push_back(std::move(c));
        return m_curveIdx[id] = static_cast<int>(m_b->curves.size()) - 1;
    }

    int surface(long long id, int depth = 0) {
        if (const auto it = m_surfIdx.find(id); it != m_surfIdx.end()) return it->second;
        const Entity* e = get(id);
        if (!e || depth > 8) return -1;
        BSurface s;
        const std::string& t = e->type();
        if (t == "RECTANGULAR_TRIMMED_SURFACE" || t == "CURVE_BOUNDED_SURFACE") {
            const Record& r = e->parts[0];
            return r.args.size() > 1 && r.args[1].isRef() ? m_surfIdx[id] = surface(r.args[1].ref, depth + 1) : -1;
        }
        if (t == "PLANE") {
            s.kind = BSurfaceKind::Plane;
            s.f = placement(e->parts[0].args[1]);
        } else if (t == "CYLINDRICAL_SURFACE") {
            s.kind = BSurfaceKind::Cylinder;
            s.f = placement(e->parts[0].args[1]);
            s.r1 = num(e->parts[0].args[2]) * m_scale;
        } else if (t == "CONICAL_SURFACE") {
            s.kind = BSurfaceKind::Cone;
            s.f = placement(e->parts[0].args[1]);
            s.r1 = num(e->parts[0].args[2]) * m_scale;
            s.r2 = num(e->parts[0].args[3]) * m_angle;
        } else if (t == "SPHERICAL_SURFACE") {
            s.kind = BSurfaceKind::Sphere;
            s.f = placement(e->parts[0].args[1]);
            s.r1 = num(e->parts[0].args[2]) * m_scale;
        } else if (t == "TOROIDAL_SURFACE" || t == "DEGENERATE_TOROIDAL_SURFACE") {
            s.kind = BSurfaceKind::Torus;
            s.f = placement(e->parts[0].args[1]);
            s.r1 = num(e->parts[0].args[2]) * m_scale;
            s.r2 = num(e->parts[0].args[3]) * m_scale;
        } else if (t == "SURFACE_OF_LINEAR_EXTRUSION") {
            const Record& r = e->parts[0];
            s.kind = BSurfaceKind::Extrusion;
            s.curve = r.args[1].isRef() ? curve(r.args[1].ref) : -1;
            s.dir = r.args[2].isRef() ? vector(r.args[2].ref) : DVec3{0, 0, 1};
            if (s.curve < 0) return -1;
        } else if (t == "SURFACE_OF_REVOLUTION") {
            const Record& r = e->parts[0];
            s.kind = BSurfaceKind::Revolution;
            s.curve = r.args[1].isRef() ? curve(r.args[1].ref) : -1;
            // AXIS1_PLACEMENT(name, location, axis)
            const Record* ax = r.args[2].isRef() ? rec(r.args[2].ref, "AXIS1_PLACEMENT") : nullptr;
            DVec3 o{0, 0, 0}, z{0, 0, 1};
            if (ax && ax->args.size() > 1 && ax->args[1].isRef()) o = point(ax->args[1].ref);
            if (ax && ax->args.size() > 2 && ax->args[2].isRef()) z = direction(ax->args[2].ref);
            s.f = Frame::make(o, z, {1, 0, 0});
            if (s.curve < 0) return -1;
        } else if (t == "B_SPLINE_SURFACE_WITH_KNOTS") {
            const Record& r = e->parts[0];
            // (name, du, dv, points, form, uclosed, vclosed, si, umults, vmults, uknots, vknots, spec)
            if (r.args.size() < 12) return -1;
            s.kind = BSurfaceKind::BSpline;
            s.bs.du = static_cast<int>(num(r.args[1]));
            s.bs.dv = static_cast<int>(num(r.args[2]));
            grid(r.args[3], s.bs);
            s.bs.ku = knots(r.args[8], r.args[10]);
            s.bs.kv = knots(r.args[9], r.args[11]);
        } else if (const Record* b = e->part("B_SPLINE_SURFACE")) {
            s.kind = BSurfaceKind::BSpline;
            if (b->args.size() < 3) return -1;
            s.bs.du = static_cast<int>(num(b->args[0]));
            s.bs.dv = static_cast<int>(num(b->args[1]));
            grid(b->args[2], s.bs);
            if (const Record* k = e->part("B_SPLINE_SURFACE_WITH_KNOTS"); k && k->args.size() >= 4) {
                s.bs.ku = knots(k->args[0], k->args[2]);
                s.bs.kv = knots(k->args[1], k->args[3]);
            }
            if (const Record* w = e->part("RATIONAL_B_SPLINE_SURFACE"); w && !w->args.empty())
                for (const Value& row : w->args[0].list)
                    for (const Value& v : row.list) s.bs.w.push_back(num(v));
        } else {
            m_rep.warnings.push_back("surface " + (t.empty() ? std::string("(complex)") : t) + " is not supported");
            return -1;
        }
        if (s.kind == BSurfaceKind::BSpline) {
            const auto& b = s.bs;
            if (b.nu < 2 || b.nv < 2 || b.ku.size() != static_cast<std::size_t>(b.nu + b.du + 1) || b.kv.size() != static_cast<std::size_t>(b.nv + b.dv + 1) ||
                (!b.w.empty() && b.w.size() != b.cp.size()))
                return -1;
        }
        m_b->surfaces.push_back(std::move(s));
        return m_surfIdx[id] = static_cast<int>(m_b->surfaces.size()) - 1;
    }
    void grid(const Value& rows, BSplineSurface& bs) const {
        bs.nu = static_cast<int>(rows.list.size());
        bs.nv = bs.nu ? static_cast<int>(rows.list[0].list.size()) : 0;
        for (const Value& row : rows.list)
            for (const Value& v : row.list) bs.cp.push_back(point(v.ref));
    }

    // --- topology ---
    int vertex(long long id) {
        if (const auto it = m_vertexIdx.find(id); it != m_vertexIdx.end()) return it->second;
        const Record* r = rec(id, "VERTEX_POINT");
        if (!r || r->args.size() < 2) return -1;
        m_b->vertices.push_back({point(r->args[1].ref)});
        return m_vertexIdx[id] = static_cast<int>(m_b->vertices.size()) - 1;
    }
    int edge(long long id) {
        if (const auto it = m_edgeIdx.find(id); it != m_edgeIdx.end()) return it->second;
        const Record* r = rec(id, "EDGE_CURVE");
        if (!r || r->args.size() < 5) return -1;
        BrepEdge e;
        e.v0 = vertex(r->args[1].ref);
        e.v1 = vertex(r->args[2].ref);
        e.curve = r->args[3].isRef() ? curve(r->args[3].ref) : -1;
        e.sameSense = flag(r->args[4]);
        if (e.v0 < 0 || e.v1 < 0) return -1;
        if (e.curve < 0 && e.v0 == e.v1) return -1;
        m_b->edges.push_back(e);
        return m_edgeIdx[id] = static_cast<int>(m_b->edges.size()) - 1;
    }
    bool loop(long long boundId, bool outerHint, BLoop& out) {
        const Entity* be = get(boundId);
        if (!be || be->parts.size() != 1) return false;
        const Record& b = be->parts[0];
        if (b.args.size() < 3) return false;
        out.outer = outerHint || b.type == "FACE_OUTER_BOUND";
        out.sense = flag(b.args[2]);
        const Entity* le = get(b.args[1].ref);
        if (!le || le->parts.size() != 1) return false;
        const Record& l = le->parts[0];
        if (l.type == "EDGE_LOOP") {
            for (const Value& v : l.args[1].list) {
                const Record* oe = rec(v.ref, "ORIENTED_EDGE");
                if (!oe || oe->args.size() < 5) return false;
                const int ei = edge(oe->args[3].ref);
                if (ei < 0) return false;
                out.edges.push_back({ei, flag(oe->args[4])});
            }
            return !out.edges.empty();
        }
        if (l.type == "VERTEX_LOOP") {
            out.vertex = vertex(l.args[1].ref);
            return out.vertex >= 0;
        }
        if (l.type == "POLY_LOOP") {
            for (const Value& v : l.args[1].list) out.poly.push_back(point(v.ref));
            return out.poly.size() >= 3;
        }
        return false;
    }
    bool face(long long id, BFace& f) {
        const Entity* fe = get(id);
        if (!fe || fe->parts.size() != 1) return false;
        const Record& r = fe->parts[0];
        if (r.args.size() < 2 || r.args[1].type != Value::List) return false;
        for (const Value& bv : r.args[1].list) {
            BLoop l;
            if (!bv.isRef() || !loop(bv.ref, false, l)) return false;
            f.loops.push_back(std::move(l));
        }
        if ((r.type == "ADVANCED_FACE" || r.type == "FACE_SURFACE") && r.args.size() >= 4) {
            f.surface = r.args[2].isRef() ? surface(r.args[2].ref) : -1;
            f.sameSense = flag(r.args[3]);
            return f.surface >= 0;
        }
        // A faceted face: the plane of its polygon (Newell's normal).
        const BLoop* poly = nullptr;
        for (const BLoop& l : f.loops)
            if (!l.poly.empty()) poly = &l;
        if (!poly) return false;
        DVec3 n{0, 0, 0}, c{0, 0, 0};
        const auto& p = poly->poly;
        for (std::size_t i = 0; i < p.size(); ++i) {
            const DVec3 &a = p[i], &b = p[(i + 1) % p.size()];
            n += DVec3{(a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y)};
            c += a;
        }
        if (!poly->sense) n = -n;
        BSurface s;
        s.kind = BSurfaceKind::Plane;
        s.f = Frame::make(c / static_cast<double>(p.size()), n, p[1] - p[0]);
        m_b->surfaces.push_back(s);
        f.surface = static_cast<int>(m_b->surfaces.size()) - 1;
        f.sameSense = true;
        return true;
    }
    bool shell(long long id, bool closed, BShell& sh) {
        const Entity* e = get(id);
        if (!e || e->parts.size() != 1) return false;
        const Record* r = &e->parts[0];
        if (r->type == "ORIENTED_CLOSED_SHELL" && r->args.size() >= 4 && r->args[2].isRef()) return shell(r->args[2].ref, closed, sh);
        if (r->args.size() < 2 || r->args[1].type != Value::List) return false;
        sh.closed = closed && r->type != "OPEN_SHELL";
        std::size_t bad = 0;
        for (const Value& fv : r->args[1].list) {
            BFace f;
            if (fv.isRef() && face(fv.ref, f)) sh.faces.push_back(std::move(f));
            else ++bad;
        }
        if (bad) m_rep.warnings.push_back(std::to_string(bad) + " face(s) of #" + std::to_string(id) + " could not be read");
        return !sh.faces.empty();
    }
    bool solid(long long id) {
        const Record* r = rec(id);
        if (!r || r->args.size() < 2) return false;
        if (r->type == "SHELL_BASED_SURFACE_MODEL") {
            for (const Value& v : r->args[1].list) {
                BShell sh;
                if (v.isRef() && shell(v.ref, false, sh)) m_b->shells.push_back(std::move(sh));
            }
            return !m_b->shells.empty();
        }
        BShell outer;
        if (!r->args[1].isRef() || !shell(r->args[1].ref, true, outer)) return false;
        m_b->shells.push_back(std::move(outer));
        if (r->type == "BREP_WITH_VOIDS" && r->args.size() >= 3)
            for (const Value& v : r->args[2].list) {
                BShell vs;
                if (v.isRef() && shell(v.ref, true, vs)) {
                    // A void's faces point into it: the shell is the oriented one, reversed.
                    const Record* oc = rec(v.ref);
                    const bool reversed = oc && oc->type == "ORIENTED_CLOSED_SHELL" && oc->args.size() >= 4 && !flag(oc->args[3]);
                    if (reversed)
                        for (BFace& f : vs.faces) f.sameSense = !f.sameSense;
                    m_b->shells.push_back(std::move(vs));
                }
            }
        return true;
    }
};

// --- writing ----------------------------------------------------------------------------

std::string esc(const std::string& s) {
    std::string o;
    for (char c : s) {
        if (c == '\'') o += "''";
        else if (static_cast<unsigned char>(c) >= 32 && static_cast<unsigned char>(c) < 127) o += c;
        else o += '_';
    }
    return o;
}

std::string real(double v) {
    if (v == 0 || std::fabs(v) < 1e-300) return "0.";
    char b[40];
    std::snprintf(b, sizeof(b), "%.15G", v);
    std::string s = b;
    if (s.find('.') == std::string::npos && s.find('E') == std::string::npos) s += ".";
    else if (s.find('.') == std::string::npos) s.insert(s.find('E'), "."); // 1E-05 -> 1.E-05
    return s;
}

class Writer {
public:
    std::string out;
    int next = 1;

    int add(const std::string& body) {
        const int id = next++;
        out += "#" + std::to_string(id) + "=" + body + ";\n";
        return id;
    }
    static std::string ref(int id) { return "#" + std::to_string(id); }
    int point(const DVec3& p) { return add("CARTESIAN_POINT('',(" + real(p.x) + "," + real(p.y) + "," + real(p.z) + "))"); }
    int dir(const DVec3& d) {
        const DVec3 n = dnormalize(d);
        return add("DIRECTION('',(" + real(n.x) + "," + real(n.y) + "," + real(n.z) + "))");
    }
    int axis(const Frame& f) { return add("AXIS2_PLACEMENT_3D(''," + ref(point(f.o)) + "," + ref(dir(f.z)) + "," + ref(dir(f.x)) + ")"); }

    static void compress(const std::vector<double>& knots, std::vector<int>& mults, std::vector<double>& uniq) {
        for (double k : knots) {
            if (!uniq.empty() && std::fabs(k - uniq.back()) < 1e-12) ++mults.back();
            else uniq.push_back(k), mults.push_back(1);
        }
    }
    template <typename T, typename F>
    static std::string list(const std::vector<T>& v, F f) {
        std::string s = "(";
        for (std::size_t i = 0; i < v.size(); ++i) s += (i ? "," : "") + f(v[i]);
        return s + ")";
    }

    int curve(const BCurve& c, const DVec3& a, const DVec3& b) {
        switch (c.kind) {
        case BCurveKind::Line: return line(c.f.o, c.f.x);
        case BCurveKind::Circle: return add("CIRCLE(''," + ref(axis(c.f)) + "," + real(c.r1) + ")");
        case BCurveKind::Ellipse: return add("ELLIPSE(''," + ref(axis(c.f)) + "," + real(c.r1) + "," + real(c.r2) + ")");
        case BCurveKind::Parabola: return add("PARABOLA(''," + ref(axis(c.f)) + "," + real(c.r1) + ")");
        case BCurveKind::Hyperbola: return add("HYPERBOLA(''," + ref(axis(c.f)) + "," + real(c.r1) + "," + real(c.r2) + ")");
        case BCurveKind::Polyline: {
            std::vector<int> ids;
            for (const DVec3& p : c.pts) ids.push_back(point(p));
            return add("POLYLINE(''," + list(ids, [](int i) { return ref(i); }) + ")");
        }
        case BCurveKind::BSpline: {
            std::vector<int> ids;
            for (const DVec3& p : c.bs.cp) ids.push_back(point(p));
            std::vector<int> mults;
            std::vector<double> uniq;
            compress(c.bs.knots, mults, uniq);
            const std::string pts = list(ids, [](int i) { return ref(i); });
            const std::string m = list(mults, [](int i) { return std::to_string(i); });
            const std::string k = list(uniq, [](double v) { return real(v); });
            if (c.bs.w.empty())
                return add("B_SPLINE_CURVE_WITH_KNOTS(''," + std::to_string(c.bs.degree) + "," + pts + ",.UNSPECIFIED.,.F.,.F.," + m + "," + k + ",.UNSPECIFIED.)");
            return add("(BOUNDED_CURVE() B_SPLINE_CURVE(" + std::to_string(c.bs.degree) + "," + pts + ",.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS(" + m + "," + k +
                       ",.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(" + list(c.bs.w, [](double v) { return real(v); }) +
                       ") REPRESENTATION_ITEM(''))");
        }
        }
        (void)a, (void)b;
        return -1;
    }
    int line(const DVec3& o, const DVec3& d) {
        const double l = dlength(d);
        return add("LINE(''," + ref(point(o)) + "," + ref(add("VECTOR(''," + ref(dir(d)) + "," + real(l > 0 ? l : 1.0) + ")")) + ")");
    }
    int surface(const BSurface& s, const std::vector<int>& curveIds) {
        switch (s.kind) {
        case BSurfaceKind::Plane: return add("PLANE(''," + ref(axis(s.f)) + ")");
        case BSurfaceKind::Cylinder: return add("CYLINDRICAL_SURFACE(''," + ref(axis(s.f)) + "," + real(s.r1) + ")");
        case BSurfaceKind::Cone: return add("CONICAL_SURFACE(''," + ref(axis(s.f)) + "," + real(s.r1) + "," + real(s.r2) + ")");
        case BSurfaceKind::Sphere: return add("SPHERICAL_SURFACE(''," + ref(axis(s.f)) + "," + real(s.r1) + ")");
        case BSurfaceKind::Torus: return add("TOROIDAL_SURFACE(''," + ref(axis(s.f)) + "," + real(s.r1) + "," + real(s.r2) + ")");
        case BSurfaceKind::Extrusion: {
            const double l = dlength(s.dir);
            return add("SURFACE_OF_LINEAR_EXTRUSION(''," + ref(curveIds[static_cast<std::size_t>(s.curve)]) + "," +
                       ref(add("VECTOR(''," + ref(dir(s.dir)) + "," + real(l) + ")")) + ")");
        }
        case BSurfaceKind::Revolution:
            return add("SURFACE_OF_REVOLUTION(''," + ref(curveIds[static_cast<std::size_t>(s.curve)]) + "," +
                       ref(add("AXIS1_PLACEMENT(''," + ref(point(s.f.o)) + "," + ref(dir(s.f.z)) + ")")) + ")");
        case BSurfaceKind::BSpline: {
            const BSplineSurface& b = s.bs;
            std::string rows = "(";
            for (int i = 0; i < b.nu; ++i) {
                rows += i ? ",(" : "(";
                for (int j = 0; j < b.nv; ++j) rows += (j ? "," : "") + ref(point(b.cp[static_cast<std::size_t>(i * b.nv + j)]));
                rows += ")";
            }
            rows += ")";
            std::vector<int> mu, mv;
            std::vector<double> ku, kv;
            compress(b.ku, mu, ku);
            compress(b.kv, mv, kv);
            const std::string ints = list(mu, [](int i) { return std::to_string(i); }) + "," + list(mv, [](int i) { return std::to_string(i); }) + "," +
                                     list(ku, [](double v) { return real(v); }) + "," + list(kv, [](double v) { return real(v); });
            if (b.w.empty())
                return add("B_SPLINE_SURFACE_WITH_KNOTS(''," + std::to_string(b.du) + "," + std::to_string(b.dv) + "," + rows + ",.UNSPECIFIED.,.F.,.F.,.F.," + ints +
                           ",.UNSPECIFIED.)");
            std::string w = "(";
            for (int i = 0; i < b.nu; ++i) {
                w += i ? ",(" : "(";
                for (int j = 0; j < b.nv; ++j) w += (j ? "," : "") + real(b.w[static_cast<std::size_t>(i * b.nv + j)]);
                w += ")";
            }
            w += ")";
            return add("(BOUNDED_SURFACE() B_SPLINE_SURFACE(" + std::to_string(b.du) + "," + std::to_string(b.dv) + "," + rows +
                       ",.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS(" + ints +
                       ",.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(" + w + ") REPRESENTATION_ITEM('') SURFACE())");
        }
        }
        return -1;
    }

    // One B-rep: returns the solid / surface model item.
    int brep(const Brep& b, bool& isSolid) {
        std::vector<int> curveIds(b.curves.size(), -1);
        for (std::size_t i = 0; i < b.curves.size(); ++i) curveIds[i] = curve(b.curves[i], {}, {});
        std::vector<int> vids(b.vertices.size());
        for (std::size_t i = 0; i < b.vertices.size(); ++i) vids[i] = add("VERTEX_POINT(''," + ref(point(b.vertices[i].p)) + ")");
        std::vector<int> eids(b.edges.size());
        for (std::size_t i = 0; i < b.edges.size(); ++i) {
            const BrepEdge& e = b.edges[i];
            int c = e.curve >= 0 ? curveIds[static_cast<std::size_t>(e.curve)] : -1;
            bool same = e.sameSense;
            if (c < 0) {
                const DVec3 p0 = b.vertices[static_cast<std::size_t>(e.v0)].p, p1 = b.vertices[static_cast<std::size_t>(e.v1)].p;
                c = line(p0, p1 - p0);
                same = true;
            }
            eids[i] = add("EDGE_CURVE(''," + ref(vids[static_cast<std::size_t>(e.v0)]) + "," + ref(vids[static_cast<std::size_t>(e.v1)]) + "," + ref(c) + "," +
                          (same ? ".T." : ".F.") + ")");
        }
        std::vector<int> sids(b.surfaces.size());
        for (std::size_t i = 0; i < b.surfaces.size(); ++i) sids[i] = surface(b.surfaces[i], curveIds);
        std::vector<int> shells;
        isSolid = true;
        for (const BShell& sh : b.shells) {
            std::vector<int> faces;
            for (const BFace& f : sh.faces) {
                std::vector<int> bounds;
                for (const BLoop& l : f.loops) {
                    int loopId;
                    if (!l.edges.empty()) {
                        std::vector<int> oes;
                        for (const OrientedEdge& oe : l.edges)
                            oes.push_back(add("ORIENTED_EDGE('',*,*," + ref(eids[static_cast<std::size_t>(oe.edge)]) + "," + (oe.forward ? ".T." : ".F.") + ")"));
                        loopId = add("EDGE_LOOP(''," + list(oes, [](int i) { return ref(i); }) + ")");
                    } else if (l.vertex >= 0) {
                        loopId = add("VERTEX_LOOP(''," + ref(vids[static_cast<std::size_t>(l.vertex)]) + ")");
                    } else {
                        std::vector<int> pts;
                        for (const DVec3& p : l.poly) pts.push_back(point(p));
                        loopId = add("POLY_LOOP(''," + list(pts, [](int i) { return ref(i); }) + ")");
                    }
                    bounds.push_back(add(std::string(l.outer ? "FACE_OUTER_BOUND" : "FACE_BOUND") + "(''," + ref(loopId) + "," + (l.sense ? ".T." : ".F.") + ")"));
                }
                faces.push_back(add("ADVANCED_FACE(''," + list(bounds, [](int i) { return ref(i); }) + "," + ref(sids[static_cast<std::size_t>(f.surface)]) + "," +
                                    (f.sameSense ? ".T." : ".F.") + ")"));
            }
            if (!sh.closed) isSolid = false;
            shells.push_back(add(std::string(sh.closed ? "CLOSED_SHELL" : "OPEN_SHELL") + "(''," + list(faces, [](int i) { return ref(i); }) + ")"));
        }
        if (isSolid && shells.size() == 1) return add("MANIFOLD_SOLID_BREP('" + esc(b.name) + "'," + ref(shells[0]) + ")");
        if (isSolid) {
            std::vector<int> voids;
            for (std::size_t i = 1; i < shells.size(); ++i) voids.push_back(add("ORIENTED_CLOSED_SHELL('',*," + ref(shells[i]) + ",.T.)"));
            return add("BREP_WITH_VOIDS('" + esc(b.name) + "'," + ref(shells[0]) + "," + list(voids, [](int i) { return ref(i); }) + ")");
        }
        isSolid = false;
        return add("SHELL_BASED_SURFACE_MODEL('" + esc(b.name) + "'," + list(shells, [](int i) { return ref(i); }) + ")");
    }
};

} // namespace

std::vector<StepPart> readStep(const std::string& text, StepReadReport* report, std::string* error) {
    StepReadReport rep;
    std::unordered_map<long long, Entity> ents;
    std::string err;
    Parser p(text);
    if (!p.parse(ents, rep.schema, err)) {
        if (error) *error = err;
        if (report) *report = rep;
        return {};
    }
    rep.entities = ents.size();
    Reader r(ents, rep);
    std::vector<StepPart> parts = r.run(err);
    if (parts.empty() && error) *error = err;
    if (report) *report = rep;
    return parts;
}

std::vector<StepPart> readStepFile(const std::string& path, StepReadReport* report, std::string* error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        if (error) *error = "Can't open " + path;
        return {};
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::vector<StepPart> parts = readStep(ss.str(), report, error);
    // Parts named after the translator that wrote the file ("Open CASCADE STEP translator
    // 7.5 1"), or not named at all, take the file's name instead.
    const std::string stem = std::filesystem::path(path).stem().string();
    for (StepPart& p : parts) {
        const bool placeholder = p.name.empty() || p.name == "STEP part" || p.name.find("STEP translator") != std::string::npos;
        if (placeholder && !stem.empty()) {
            p.name = stem;
            if (p.brep) p.brep->name = stem;
        }
    }
    return parts;
}

std::string writeStep(const std::vector<StepPart>& parts, const std::string& fileName) {
    Writer w;
    // Contexts and units: millimetres, radians, steradians, and a model tolerance.
    const int app = w.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')");
    w.add("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000," + Writer::ref(app) + ")");
    const int pctx = w.add("PRODUCT_CONTEXT(''," + Writer::ref(app) + ",'mechanical')");
    const int pdctx = w.add("PRODUCT_DEFINITION_CONTEXT('part definition'," + Writer::ref(app) + ",'design')");
    const int mm = w.add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))");
    const int rad = w.add("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))");
    const int sr = w.add("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())");
    const int unc = w.add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06)," + Writer::ref(mm) + ",'distance_accuracy_value','confusion accuracy')");
    const int ctx = w.add("(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((" + Writer::ref(unc) + ")) GLOBAL_UNIT_ASSIGNED_CONTEXT((" +
                          Writer::ref(mm) + "," + Writer::ref(rad) + "," + Writer::ref(sr) + ")) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY'))");
    std::vector<int> prods;
    // Sub-assemblies: a product each, with an empty shape the parts are placed in.
    struct Node {
        int pd = 0, rep = 0;
    };
    std::map<std::string, Node> nodes;
    int nauos = 0;
    auto place = [&](const std::string& parentPath, int childPd, int childRep, const std::string& childName) {
        const Node& parent = nodes.at(parentPath);
        const std::string id = "NAUO" + std::to_string(++nauos);
        const int nauo = w.add("NEXT_ASSEMBLY_USAGE_OCCURRENCE('" + id + "','" + esc(childName) + "',''," + Writer::ref(parent.pd) + "," + Writer::ref(childPd) + ",$)");
        const int pds = w.add("PRODUCT_DEFINITION_SHAPE('Placement','Placement of " + esc(childName) + "'," + Writer::ref(nauo) + ")");
        // The parts are written where they are, so the placement is the identity.
        const int idt = w.add("ITEM_DEFINED_TRANSFORMATION('',''," + Writer::ref(w.axis(Frame{})) + "," + Writer::ref(w.axis(Frame{})) + ")");
        const int rr = w.add("(REPRESENTATION_RELATIONSHIP(''," + std::string("''") + "," + Writer::ref(childRep) + "," + Writer::ref(parent.rep) +
                             ")REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(" + Writer::ref(idt) + ")SHAPE_REPRESENTATION_RELATIONSHIP())");
        w.add("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(" + Writer::ref(rr) + "," + Writer::ref(pds) + ")");
    };
    std::function<void(const std::string&)> addNode = [&](const std::string& path) {
        if (path.empty() || nodes.count(path)) return;
        const std::size_t slash = path.rfind('/');
        const std::string parent = slash == std::string::npos ? "" : path.substr(0, slash);
        addNode(parent);
        const std::string nm = esc(slash == std::string::npos ? path : path.substr(slash + 1));
        const int prod = w.add("PRODUCT('" + nm + "','" + nm + "',''," + "(" + Writer::ref(pctx) + "))");
        prods.push_back(prod);
        const int pdf = w.add("PRODUCT_DEFINITION_FORMATION(''," + std::string("''") + "," + Writer::ref(prod) + ")");
        const int pd = w.add("PRODUCT_DEFINITION('design',''," + Writer::ref(pdf) + "," + Writer::ref(pdctx) + ")");
        const int pds = w.add("PRODUCT_DEFINITION_SHAPE('',''," + Writer::ref(pd) + ")");
        const int rep = w.add("SHAPE_REPRESENTATION('" + nm + "',(" + Writer::ref(w.axis(Frame{})) + ")," + Writer::ref(ctx) + ")");
        w.add("SHAPE_DEFINITION_REPRESENTATION(" + Writer::ref(pds) + "," + Writer::ref(rep) + ")");
        nodes[path] = {pd, rep};
        if (!parent.empty()) place(parent, pd, rep, slash == std::string::npos ? path : path.substr(slash + 1));
    };
    for (const StepPart& part : parts) {
        if (!part.brep || part.brep->empty()) continue;
        addNode(part.assembly);
        const std::string nm = esc(part.name.empty() ? "Part" : part.name);
        const int prod = w.add("PRODUCT('" + nm + "','" + nm + "',''," + "(" + Writer::ref(pctx) + "))");
        prods.push_back(prod);
        const int pdf = w.add("PRODUCT_DEFINITION_FORMATION(''," + std::string("''") + "," + Writer::ref(prod) + ")");
        const int pd = w.add("PRODUCT_DEFINITION('design',''," + Writer::ref(pdf) + "," + Writer::ref(pdctx) + ")");
        const int pds = w.add("PRODUCT_DEFINITION_SHAPE('',''," + Writer::ref(pd) + ")");
        Brep b = *part.brep;
        b.name = part.name;
        bool solid = true;
        const int item = w.brep(b, solid);
        const int origin = w.axis(Frame{});
        const int rep = w.add(std::string(solid ? "ADVANCED_BREP_SHAPE_REPRESENTATION" : "MANIFOLD_SURFACE_SHAPE_REPRESENTATION") + "('" + nm + "',(" +
                              Writer::ref(item) + "," + Writer::ref(origin) + ")," + Writer::ref(ctx) + ")");
        w.add("SHAPE_DEFINITION_REPRESENTATION(" + Writer::ref(pds) + "," + Writer::ref(rep) + ")");
        if (!part.assembly.empty()) place(part.assembly, pd, rep, part.name.empty() ? "Part" : part.name);
    }
    if (!prods.empty()) {
        std::string l = "(";
        for (std::size_t i = 0; i < prods.size(); ++i) l += (i ? "," : "") + Writer::ref(prods[i]);
        w.add("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$," + l + "))");
    }
    std::string head = "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('SI-CDM model'),'2;1');\nFILE_NAME('" + esc(fileName) +
                       "','2026-01-01T00:00:00',(''),(''),'SI-CDM','SI-CDM','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n";
    return head + w.out + "ENDSEC;\nEND-ISO-10303-21;\n";
}

bool writeStepFile(const std::vector<StepPart>& parts, const std::string& path, std::string* error) {
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        if (error) *error = "Can't write " + path;
        return false;
    }
    const std::size_t slash = path.find_last_of("/\\");
    f << writeStep(parts, slash == std::string::npos ? path : path.substr(slash + 1));
    return static_cast<bool>(f);
}

std::shared_ptr<Brep> primitiveBrep(const std::string& k, const std::vector<float>& pf) {
    // The dimensions as typed (see decimal()), in double.
    std::vector<double> p;
    for (float v : pf) p.push_back(decimal(v));
    auto need = [&](std::size_t n) { return p.size() >= n; };
    if (k == "box" && need(6)) {
        const DVec3 lo{std::min(p[0], p[3]), std::min(p[1], p[4]), std::min(p[2], p[5])};
        const DVec3 hi{std::max(p[0], p[3]), std::max(p[1], p[4]), std::max(p[2], p[5])};
        if (hi.x - lo.x <= 0 || hi.y - lo.y <= 0 || hi.z - lo.z <= 0) return nullptr;
        return brepBox(lo, hi);
    }
    if (k == "cylinder" && need(2) && p[0] > 0 && p[1] > 0) return brepCylinder({0, 0, 0}, p[0], p[1]);
    if (k == "cone" && need(3) && p[0] > 0 && p[1] >= 0 && p[2] > 0) return brepCone({0, 0, 0}, p[0], p[1], p[2]);
    if (k == "sphere" && need(1) && p[0] > 0) return brepSphere({0, 0, p[0]}, p[0]);
    if (k == "torus" && need(2) && p[0] > 0 && p[1] > 0 && p[1] < p[0]) return brepTorus({0, 0, p[1]}, p[0], p[1]);
    if (k == "extrude" && need(7)) {
        const MeshData m = buildExtrusion(pf);
        if (m.indices.empty()) return nullptr;
        if (p[3] != 1.0) return meshToBrep(m); // a polygon's prism: flat faces
        const PlaneFrame f = planeFrame(static_cast<SketchPlane>(static_cast<int>(p[0])));
        const DVec3 u(f.u), v(f.v), n(f.normal), o(f.origin);
        auto b = brepCylinder({0, 0, 0}, p[6], p[2] - p[1]);
        b->transform(Placement::fromFrames(Frame{}, Frame::make(o + u * p[4] + v * p[5] + n * p[1], n, u)));
        return b;
    }
    // Flat-faced shapes: their facets are exact.
    if (k == "wedge" || k == "pyramid" || k == "prism") {
        const MeshData m = buildPrimitive(k, pf);
        if (m.indices.empty()) return nullptr;
        return meshToBrep(m);
    }
    return nullptr;
}

void attachPrimitiveBrep(Body& b) {
    b.brep.reset();
    b.brepKey = 0;
    if (b.kind == "mesh" || b.mesh.indices.empty()) return;
    std::shared_ptr<Brep> br = primitiveBrep(b.kind, b.params);
    if (!br) return;
    // The mesh is rotated, then scaled (see Body::rebuild): the same for the B-rep, in double.
    Placement t = eulerPlacement(decimal(b.rotation));
    t.s = decimal(b.scale);
    br->transform(t);
    br->name = b.name;
    b.brep = std::move(br);
    b.brepKey = meshKey(b.mesh);
}

std::shared_ptr<const Brep> exactBrep(const Body& b) {
    if (b.exact()) return b.brep;
    if (b.kind == "mesh" || !primitiveBrep(b.kind, b.params)) return nullptr;
    Body copy = b;
    copy.rebuild();
    if (copy.exact() && copy.brepKey == meshKey(b.mesh)) return copy.brep;
    return nullptr;
}

// --- meshes as planar B-reps ----------------------------------------------------------------

std::shared_ptr<Brep> meshToBrep(const MeshData& mesh, double planarTolerance) {
    auto b = std::make_shared<Brep>();
    // Weld positions.
    std::map<std::tuple<float, float, float>, int> weld;
    std::vector<int> vid(mesh.vertices.size());
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        const Vec3& p = mesh.vertices[i].position;
        const auto [it, added] = weld.try_emplace({p.x + 0.0f, p.y + 0.0f, p.z + 0.0f}, static_cast<int>(b->vertices.size()));
        if (added) b->vertices.push_back({DVec3(p)});
        vid[i] = it->second;
    }
    // Seams a hair apart (a round primitive's 0 and 360 degree meridian) close: vertices on
    // open edges within a millionth of the size snap together.
    {
        std::map<std::pair<int, int>, int> uses;
        for (std::size_t k = 0; k + 2 < mesh.indices.size(); k += 3)
            for (int e = 0; e < 3; ++e) {
                int a = vid[mesh.indices[k + static_cast<std::size_t>(e)]], c = vid[mesh.indices[k + static_cast<std::size_t>((e + 1) % 3)]];
                if (a == c) continue;
                if (a > c) std::swap(a, c);
                ++uses[{a, c}];
            }
        std::vector<int> open;
        for (const auto& [e, n] : uses)
            if (n == 1) open.push_back(e.first), open.push_back(e.second);
        std::sort(open.begin(), open.end());
        open.erase(std::unique(open.begin(), open.end()), open.end());
        if (!open.empty()) {
            double size = 0;
            for (const BrepVertex& v : b->vertices) size = std::max({size, std::fabs(v.p.x), std::fabs(v.p.y), std::fabs(v.p.z)});
            const double tol = std::max(size, 1.0) * 1e-6;
            std::vector<int> rep(b->vertices.size());
            for (std::size_t i = 0; i < rep.size(); ++i) rep[i] = static_cast<int>(i);
            std::map<std::tuple<long long, long long, long long>, std::vector<int>> grid;
            auto cellOf = [&](const DVec3& p) { return std::make_tuple(std::llround(p.x / (2 * tol)), std::llround(p.y / (2 * tol)), std::llround(p.z / (2 * tol))); };
            for (int v : open) {
                const DVec3& p = b->vertices[static_cast<std::size_t>(v)].p;
                const auto [cx, cy, cz] = cellOf(p);
                int found = -1;
                for (long long dx = -1; dx <= 1 && found < 0; ++dx)
                    for (long long dy = -1; dy <= 1 && found < 0; ++dy)
                        for (long long dz = -1; dz <= 1 && found < 0; ++dz) {
                            const auto it = grid.find({cx + dx, cy + dy, cz + dz});
                            if (it == grid.end()) continue;
                            for (int o : it->second)
                                if (dlength(b->vertices[static_cast<std::size_t>(o)].p - p) <= tol) {
                                    found = o;
                                    break;
                                }
                        }
                if (found >= 0) rep[static_cast<std::size_t>(v)] = found;
                else grid[{cx, cy, cz}].push_back(v);
            }
            for (int& v : vid) v = rep[static_cast<std::size_t>(v)];
        }
    }
    struct Tri {
        int v[3];
        DVec3 n;
        double d;
    };
    std::vector<Tri> tris;
    for (std::size_t k = 0; k + 2 < mesh.indices.size(); k += 3) {
        Tri t{{vid[mesh.indices[k]], vid[mesh.indices[k + 1]], vid[mesh.indices[k + 2]]}, {}, 0};
        if (t.v[0] == t.v[1] || t.v[1] == t.v[2] || t.v[2] == t.v[0]) continue;
        const DVec3 &a = b->vertices[static_cast<std::size_t>(t.v[0])].p, &c1 = b->vertices[static_cast<std::size_t>(t.v[1])].p,
                    &c2 = b->vertices[static_cast<std::size_t>(t.v[2])].p;
        const DVec3 n = dcross(c1 - a, c2 - a);
        if (dlength(n) < 1e-18) continue;
        t.n = dnormalize(n);
        t.d = ddot(t.n, a);
        tris.push_back(t);
    }
    // Edge -> triangles.
    std::map<std::pair<int, int>, std::vector<int>> edgeTris;
    for (int i = 0; i < static_cast<int>(tris.size()); ++i)
        for (int e = 0; e < 3; ++e) {
            int a = tris[static_cast<std::size_t>(i)].v[e], c = tris[static_cast<std::size_t>(i)].v[(e + 1) % 3];
            if (a > c) std::swap(a, c);
            edgeTris[{a, c}].push_back(i);
        }
    // Regions of coplanar neighbours.
    double size = 0;
    for (const BrepVertex& v : b->vertices) size = std::max({size, std::fabs(v.p.x), std::fabs(v.p.y), std::fabs(v.p.z)});
    const double tol = planarTolerance * std::max(size, 1.0);
    std::vector<int> region(tris.size(), -1);
    int regions = 0;
    for (int s = 0; s < static_cast<int>(tris.size()); ++s) {
        if (region[static_cast<std::size_t>(s)] >= 0) continue;
        const Tri& seed = tris[static_cast<std::size_t>(s)];
        std::vector<int> stack{s};
        region[static_cast<std::size_t>(s)] = regions;
        while (!stack.empty()) {
            const int t = stack.back();
            stack.pop_back();
            for (int e = 0; e < 3; ++e) {
                int a = tris[static_cast<std::size_t>(t)].v[e], c = tris[static_cast<std::size_t>(t)].v[(e + 1) % 3];
                if (a > c) std::swap(a, c);
                const auto& nb = edgeTris[{a, c}];
                if (nb.size() != 2) continue;
                for (int o : nb) {
                    if (region[static_cast<std::size_t>(o)] >= 0) continue;
                    const Tri& T = tris[static_cast<std::size_t>(o)];
                    if (ddot(T.n, seed.n) < 1 - 1e-9) continue;
                    bool flat = true;
                    for (int v : T.v) flat = flat && std::fabs(ddot(seed.n, b->vertices[static_cast<std::size_t>(v)].p) - seed.d) <= tol;
                    if (!flat) continue;
                    region[static_cast<std::size_t>(o)] = regions;
                    stack.push_back(o);
                }
            }
        }
        ++regions;
    }
    // Boundary edges of each region become B-rep edges (shared by the regions on both sides).
    std::map<std::pair<int, int>, int> edgeId;
    auto edgeOf = [&](int a, int c) {
        const std::pair<int, int> key{std::min(a, c), std::max(a, c)};
        const auto it = edgeId.find(key);
        if (it != edgeId.end()) return it->second;
        b->edges.push_back({key.first, key.second, -1, true});
        return edgeId[key] = static_cast<int>(b->edges.size()) - 1;
    };
    BShell sh;
    bool closed = true;
    for (const auto& [k, v] : edgeTris)
        if (v.size() != 2) closed = false;
    sh.closed = closed;
    std::vector<std::vector<int>> members(static_cast<std::size_t>(regions));
    for (int i = 0; i < static_cast<int>(tris.size()); ++i) members[static_cast<std::size_t>(region[static_cast<std::size_t>(i)])].push_back(i);
    for (int r = 0; r < regions; ++r) {
        // Directed boundary edges (a -> c, as the region's triangles run).
        std::multimap<int, int> next;
        for (int t : members[static_cast<std::size_t>(r)])
            for (int e = 0; e < 3; ++e) {
                const int a = tris[static_cast<std::size_t>(t)].v[e], c = tris[static_cast<std::size_t>(t)].v[(e + 1) % 3];
                const auto& nb = edgeTris[{std::min(a, c), std::max(a, c)}];
                bool inner = nb.size() == 2 && region[static_cast<std::size_t>(nb[0])] == r && region[static_cast<std::size_t>(nb[1])] == r;
                if (!inner) next.insert({a, c});
            }
        BFace f;
        BSurface s;
        s.kind = BSurfaceKind::Plane;
        const Tri& t0 = tris[static_cast<std::size_t>(members[static_cast<std::size_t>(r)][0])];
        const DVec3 o = b->vertices[static_cast<std::size_t>(t0.v[0])].p;
        s.f = Frame::make(o, t0.n, b->vertices[static_cast<std::size_t>(t0.v[1])].p - o);
        b->surfaces.push_back(s);
        f.surface = static_cast<int>(b->surfaces.size()) - 1;
        f.sameSense = true;
        // Chain the directed edges into loops.
        double bestArea = -1;
        while (!next.empty()) {
            BLoop l;
            const int start = next.begin()->first;
            int at = start;
            std::vector<int> verts;
            for (int guard = 0; guard < 1000000; ++guard) {
                const auto it = next.find(at);
                if (it == next.end()) break;
                const int to = it->second;
                next.erase(it);
                l.edges.push_back({edgeOf(at, to), at < to});
                verts.push_back(at);
                at = to;
                if (at == start) break;
            }
            if (l.edges.size() < 3) continue;
            // The outer loop has the largest area (along the normal).
            DVec3 ar{0, 0, 0};
            for (std::size_t i = 0; i < verts.size(); ++i)
                ar += dcross(b->vertices[static_cast<std::size_t>(verts[i])].p, b->vertices[static_cast<std::size_t>(verts[(i + 1) % verts.size()])].p);
            const double area = ddot(ar, t0.n);
            if (area > bestArea) {
                bestArea = area;
                for (BLoop& q : f.loops) q.outer = false;
                l.outer = true;
            }
            f.loops.push_back(std::move(l));
        }
        if (!f.loops.empty()) sh.faces.push_back(std::move(f));
    }
    b->shells.push_back(std::move(sh));
    return b;
}

} // namespace sicdm

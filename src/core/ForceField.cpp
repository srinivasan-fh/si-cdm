// The robot sandbox's forces (see ForceField.hpp).

#include "core/ForceField.hpp"

#include "core/Rig.hpp"
#include "core/Scene.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>

namespace sicdm {

namespace {

constexpr double kPiD = 3.14159265358979323846;

// Opcodes: values, then operators and functions with their argument counts.
enum Code : int {
    kNumber,
    kVariable, // value = index
    kAdd, kSub, kMul, kDiv, kPow, kNeg,
    kSin, kCos, kTan, kAsin, kAcos, kAtan, kExp, kLog, kSqrt, kAbs, kSign, kStep,
    kAtan2, kMin, kMax, kPowF,
    kClamp,
};

struct Function {
    const char* name;
    int code, args;
};
constexpr Function kFunctions[] = {{"sin", kSin, 1},   {"cos", kCos, 1},   {"tan", kTan, 1},   {"asin", kAsin, 1},   {"acos", kAcos, 1},
                                   {"atan", kAtan, 1}, {"exp", kExp, 1},   {"log", kLog, 1},   {"sqrt", kSqrt, 1},   {"abs", kAbs, 1},
                                   {"sign", kSign, 1}, {"step", kStep, 1}, {"atan2", kAtan2, 2}, {"min", kMin, 2}, {"max", kMax, 2},
                                   {"pow", kPowF, 2},  {"clamp", kClamp, 3}};

struct Parser {
    const std::string& s;
    std::size_t i = 0;
    std::vector<std::pair<int, double>>& out;
    std::string error;

    void skip() {
        while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    }
    bool eat(char c) {
        skip();
        if (i < s.size() && s[i] == c) {
            ++i;
            return true;
        }
        return false;
    }
    bool fail(const std::string& why) {
        if (error.empty()) error = why;
        return false;
    }
    bool expr() {
        if (!term()) return false;
        for (;;) {
            if (eat('+')) {
                if (!term()) return false;
                out.push_back({kAdd, 0});
            } else if (eat('-')) {
                if (!term()) return false;
                out.push_back({kSub, 0});
            } else {
                return true;
            }
        }
    }
    bool term() {
        if (!unary()) return false;
        for (;;) {
            if (eat('*')) {
                if (!unary()) return false;
                out.push_back({kMul, 0});
            } else if (eat('/')) {
                if (!unary()) return false;
                out.push_back({kDiv, 0});
            } else {
                return true;
            }
        }
    }
    bool unary() {
        if (eat('-')) {
            if (!unary()) return false;
            out.push_back({kNeg, 0});
            return true;
        }
        if (eat('+')) return unary();
        return power();
    }
    bool power() {
        if (!primary()) return false;
        if (eat('^')) {
            if (!unary()) return false; // right-associative: 2^3^2 = 2^9
            out.push_back({kPow, 0});
        }
        return true;
    }
    bool primary() {
        skip();
        if (i >= s.size()) return fail("the expression ends too soon");
        if (eat('(')) {
            if (!expr()) return false;
            return eat(')') ? true : fail("a ( has no )");
        }
        const char c = s[i];
        if (std::isdigit(static_cast<unsigned char>(c)) || c == '.') {
            const char* begin = s.c_str() + i;
            char* end = nullptr;
            const double v = std::strtod(begin, &end);
            if (end == begin) return fail("a bad number");
            i += static_cast<std::size_t>(end - begin);
            out.push_back({kNumber, v});
            return true;
        }
        if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
            std::string name;
            while (i < s.size() && (std::isalnum(static_cast<unsigned char>(s[i])) || s[i] == '_'))
                name += static_cast<char>(std::tolower(static_cast<unsigned char>(s[i++])));
            if (eat('(')) {
                for (const Function& f : kFunctions) {
                    if (name != f.name) continue;
                    for (int a = 0; a < f.args; ++a) {
                        if (a > 0 && !eat(',')) return fail(name + " takes " + std::to_string(f.args) + " arguments");
                        if (!expr()) return false;
                    }
                    if (!eat(')')) return fail(name + " takes " + std::to_string(f.args) + " argument(s)");
                    out.push_back({f.code, 0});
                    return true;
                }
                return fail("no function named " + name);
            }
            if (name == "pi") {
                out.push_back({kNumber, kPiD});
                return true;
            }
            const auto& vars = ForceExpression::variables();
            for (std::size_t v = 0; v < vars.size(); ++v)
                if (vars[v] == name) {
                    out.push_back({kVariable, static_cast<double>(v)});
                    return true;
                }
            return fail("no variable named " + name + " (t, x, y, z, vx, vy, vz, wx, wy, wz, mass, g)");
        }
        return fail(std::string("unexpected '") + c + "'");
    }
};

std::mutex& scriptLock() {
    static std::mutex m;
    return m;
}
std::map<std::string, ForceFunction>& scripts() {
    static std::map<std::string, ForceFunction> s;
    return s;
}

V3d sub3(const V3d& a, const V3d& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3d add3(const V3d& a, const V3d& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V3d mul3(const V3d& a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
V3d cross3(const V3d& a, const V3d& b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
double norm3(const V3d& a) { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }
V3d rotate(const LinkPose& p, const V3d& v) {
    return {p.R[0] * v[0] + p.R[1] * v[1] + p.R[2] * v[2], p.R[3] * v[0] + p.R[4] * v[1] + p.R[5] * v[2], p.R[6] * v[0] + p.R[7] * v[1] + p.R[8] * v[2]};
}
V3d metres(const Vec3& mm) { return {mm.x * 1e-3, mm.y * 1e-3, mm.z * 1e-3}; }

// Where a part is at rest, as samples spread through its box (each the centre of a cell with
// an equal share of its volume): buoyancy and magnets act on each.
constexpr int kCells = 4; // per side
std::vector<V3d> partSamples(const Body& b, V3d* cellHalf = nullptr) {
    const AABB bb = b.bounds();
    std::vector<V3d> out;
    if (!bb.valid()) return out;
    constexpr int n = kCells;
    if (cellHalf) *cellHalf = {(bb.max.x - bb.min.x) * 0.5e-3 / n, (bb.max.y - bb.min.y) * 0.5e-3 / n, (bb.max.z - bb.min.z) * 0.5e-3 / n};
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            for (int k = 0; k < n; ++k)
                out.push_back({(bb.min.x + (bb.max.x - bb.min.x) * (i + 0.5) / n) * 1e-3, (bb.min.y + (bb.max.y - bb.min.y) * (j + 0.5) / n) * 1e-3,
                               (bb.min.z + (bb.max.z - bb.min.z) * (k + 0.5) / n) * 1e-3});
    return out;
}

// Velocity of a world point carried by a link.
V3d pointVelocity(const Multibody& m, int link, const V3d& p) {
    return add3(m.linkVelocity(link), cross3(m.linkAngularVelocity(link), sub3(p, m.linkCom(link))));
}

bool activeAt(const ForceSpec& f, double t) { return t + 1e-12 >= f.from && (f.until < 0 || t <= f.until); }

} // namespace

const std::vector<std::string>& ForceExpression::variables() {
    static const std::vector<std::string> v{"t", "x", "y", "z", "vx", "vy", "vz", "wx", "wy", "wz", "mass", "g"};
    return v;
}

bool ForceExpression::compile(const std::string& text, std::string* error) {
    m_ops.clear();
    std::vector<std::pair<int, double>> ops;
    Parser p{text, 0, ops, {}};
    bool ok = p.expr();
    p.skip();
    if (ok && p.i < text.size()) ok = p.fail(std::string("unexpected '") + text[p.i] + "'");
    if (!ok) {
        if (error) *error = p.error + " in \"" + text + "\"";
        return false;
    }
    for (const auto& [c, v] : ops) m_ops.push_back({c, v});
    return true;
}

double ForceExpression::eval(const double* vars) const {
    double st[64];
    int n = 0;
    for (const Op& op : m_ops) {
        if (n >= 62) return 0; // deeper than any sensible expression
        switch (op.code) {
        case kNumber: st[n++] = op.value; break;
        case kVariable: st[n++] = vars[static_cast<int>(op.value)]; break;
        case kNeg: st[n - 1] = -st[n - 1]; break;
        case kAdd: --n, st[n - 1] += st[n]; break;
        case kSub: --n, st[n - 1] -= st[n]; break;
        case kMul: --n, st[n - 1] *= st[n]; break;
        case kDiv: --n, st[n - 1] = st[n] != 0 ? st[n - 1] / st[n] : 0.0; break;
        case kPow:
        case kPowF: --n, st[n - 1] = std::pow(st[n - 1], st[n]); break;
        case kSin: st[n - 1] = std::sin(st[n - 1]); break;
        case kCos: st[n - 1] = std::cos(st[n - 1]); break;
        case kTan: st[n - 1] = std::tan(st[n - 1]); break;
        case kAsin: st[n - 1] = std::asin(std::clamp(st[n - 1], -1.0, 1.0)); break;
        case kAcos: st[n - 1] = std::acos(std::clamp(st[n - 1], -1.0, 1.0)); break;
        case kAtan: st[n - 1] = std::atan(st[n - 1]); break;
        case kExp: st[n - 1] = std::exp(std::min(st[n - 1], 700.0)); break;
        case kLog: st[n - 1] = st[n - 1] > 0 ? std::log(st[n - 1]) : 0.0; break;
        case kSqrt: st[n - 1] = std::sqrt(std::max(st[n - 1], 0.0)); break;
        case kAbs: st[n - 1] = std::fabs(st[n - 1]); break;
        case kSign: st[n - 1] = st[n - 1] > 0 ? 1.0 : st[n - 1] < 0 ? -1.0 : 0.0; break;
        case kStep: st[n - 1] = st[n - 1] >= 0 ? 1.0 : 0.0; break;
        case kAtan2: --n, st[n - 1] = std::atan2(st[n - 1], st[n]); break;
        case kMin: --n, st[n - 1] = std::min(st[n - 1], st[n]); break;
        case kMax: --n, st[n - 1] = std::max(st[n - 1], st[n]); break;
        case kClamp: n -= 2, st[n - 1] = std::clamp(st[n - 1], std::min(st[n], st[n + 1]), std::max(st[n], st[n + 1])); break;
        default: break;
        }
    }
    const double v = n > 0 ? st[n - 1] : 0.0;
    return std::isfinite(v) ? v : 0.0;
}

void registerForceScript(const std::string& name, ForceFunction fn) {
    std::lock_guard<std::mutex> g(scriptLock());
    scripts()[name] = std::move(fn);
}

void unregisterForceScript(const std::string& name) {
    std::lock_guard<std::mutex> g(scriptLock());
    scripts().erase(name);
}

bool hasForceScript(const std::string& name) {
    std::lock_guard<std::mutex> g(scriptLock());
    return scripts().count(name) > 0;
}

std::vector<std::string> addRobotForces(Multibody& mb, const Scene& scene, const RigSettings& settings) {
    std::vector<std::string> notes;
    const RobotRig& rig = mb.rig();
    const auto& bodies = scene.bodies();
    auto bodyNamed = [&](const std::string& n) -> int {
        for (std::size_t b = 0; b < bodies.size(); ++b)
            if (bodies[b].name == n) return static_cast<int>(b);
        return -1;
    };
    for (const ForceSpec& f : settings.forces) {
        const int b = bodyNamed(f.body);
        const int link = b >= 0 ? rig.linkOf(static_cast<std::size_t>(b)) : -1;
        if (link < 0) {
            notes.push_back("force " + f.name + ": " + f.body + " isn't a part of the robot");
            continue;
        }
        const Body& body = bodies[static_cast<std::size_t>(b)];
        const V3d at = metres(f.at);
        switch (f.kind) {
        case ForceKind::Wrench: {
            const V3d force{f.force.x, f.force.y, f.force.z}, T{f.torque.x, f.torque.y, f.torque.z};
            mb.addForceHook([f, link, at, force, T](Multibody& m) {
                if (!activeAt(f, m.time())) return;
                const LinkPose p = m.linkPose(link);
                m.applyWrench(link, f.local ? rotate(p, force) : force, f.local ? rotate(p, T) : T, p.apply(at));
            });
            break;
        }
        case ForceKind::Buoyancy: {
            V3d cell{};
            const std::vector<V3d> samples = partSamples(body, &cell);
            if (samples.empty()) break;
            const MassInertia mi = meshInertia(body, 1000.0);
            const double share = mi.volume / static_cast<double>(samples.size()); // m^3 each
            const double level = f.level * 1e-3, rho = f.density, drag = f.drag / static_cast<double>(samples.size());
            mb.addForceHook([f, link, samples, cell, share, level, rho, drag](Multibody& m) {
                if (!activeAt(f, m.time())) return;
                const V3d g = m.options().gravity;
                const double gn = norm3(g);
                if (gn <= 0 && drag <= 0) return;
                const V3d up = gn > 0 ? mul3(g, -1.0 / gn) : V3d{0, 0, 1};
                const LinkPose p = m.linkPose(link);
                const V3d ref = m.linkCom(link);
                // Each cell counts by how much of its height (as it is turned) is under the surface.
                const double halfHeight = std::max(std::fabs(p.R[6]) * cell[0] + std::fabs(p.R[7]) * cell[1] + std::fabs(p.R[8]) * cell[2], 1e-9);
                V3d F{0, 0, 0}, T{0, 0, 0};
                for (const V3d& s : samples) {
                    const V3d w = p.apply(s);
                    const double wet = std::clamp((level - w[2]) / (2 * halfHeight) + 0.5, 0.0, 1.0);
                    if (wet <= 0) continue;
                    V3d fi = mul3(up, rho * gn * share * wet);
                    if (drag > 0) fi = add3(fi, mul3(pointVelocity(m, link, w), -drag * wet));
                    F = add3(F, fi);
                    T = add3(T, cross3(sub3(w, ref), fi));
                }
                if (F[0] != 0 || F[1] != 0 || F[2] != 0) m.applyWrench(link, F, T, ref);
            });
            break;
        }
        case ForceKind::Magnet: {
            V3d cell{};
            const std::vector<V3d> samples = partSamples(body, &cell);
            if (samples.empty()) break;
            // STRENGTH is the pull with a face flush against the steel: spread over one face's
            // cells, each pulled by how near its own face is, and never more than that in all.
            const double face = static_cast<double>(kCells * kCells);
            const double per = f.strength / face, range = std::max(f.range * 1e-3, 1e-6);
            // The steel it sticks to: the ground, or a part's box where it is at rest.
            bool toPart = false;
            V3d lo{}, hi{};
            if (!f.onto.empty()) {
                const int ob = bodyNamed(f.onto);
                if (ob < 0) {
                    notes.push_back("force " + f.name + ": no part named " + f.onto);
                    break;
                }
                const AABB bb = bodies[static_cast<std::size_t>(ob)].bounds();
                lo = metres(bb.min), hi = metres(bb.max), toPart = true;
            }
            mb.addForceHook([f, link, samples, cell, per, face, range, toPart, lo, hi](Multibody& m) {
                if (!activeAt(f, m.time())) return;
                const LinkPose p = m.linkPose(link);
                const V3d ref = m.linkCom(link);
                const Ground& ground = m.options().ground;
                if (!toPart && !ground.height) return;
                // How far a cell reaches from its centre toward `dir`, as the part is turned.
                auto reach = [&](const V3d& dir) {
                    double r = 0;
                    for (std::size_t k = 0; k < 3; ++k) r += std::fabs(p.R[k] * dir[0] + p.R[3 + k] * dir[1] + p.R[6 + k] * dir[2]) * cell[k];
                    return r;
                };
                V3d F{0, 0, 0}, T{0, 0, 0};
                double pulled = 0;
                for (const V3d& s : samples) {
                    const V3d w = p.apply(s);
                    double d = 0;
                    V3d dir{0, 0, -1};
                    if (toPart) {
                        const V3d c{std::clamp(w[0], lo[0], hi[0]), std::clamp(w[1], lo[1], hi[1]), std::clamp(w[2], lo[2], hi[2])};
                        const V3d to = sub3(c, w);
                        d = norm3(to);
                        if (d < 1e-9) continue; // its centre inside the steel: the contact holds it
                        dir = mul3(to, 1.0 / d);
                    } else {
                        if (ground.normal) dir = mul3(ground.normal(w[0], w[1]), -1.0);
                        d = w[2] - ground.height(w[0], w[1]);
                    }
                    d = std::max(0.0, d - reach(dir));
                    if (d >= range) continue;
                    const double k = (1 - d / range) * (1 - d / range);
                    const V3d fi = mul3(dir, per * k);
                    pulled += k;
                    F = add3(F, fi);
                    T = add3(T, cross3(sub3(w, ref), fi));
                }
                if (pulled > face) F = mul3(F, face / pulled), T = mul3(T, face / pulled);
                if (pulled > 0) m.applyWrench(link, F, T, ref);
            });
            break;
        }
        case ForceKind::Script: {
            if (!f.function.empty()) {
                ForceFunction fn;
                {
                    std::lock_guard<std::mutex> g(scriptLock());
                    const auto it = scripts().find(f.function);
                    if (it != scripts().end()) fn = it->second;
                }
                if (!fn) {
                    notes.push_back("force " + f.name + ": no C++ force registered as " + f.function);
                    break;
                }
                const double mass = rig.links[static_cast<std::size_t>(link)].mass;
                mb.addForceHook([f, link, at, fn, mass](Multibody& m) {
                    if (!activeAt(f, m.time())) return;
                    const LinkPose p = m.linkPose(link);
                    ForceContext ctx;
                    ctx.t = m.time(), ctx.link = link, ctx.point = p.apply(at), ctx.velocity = pointVelocity(m, link, ctx.point);
                    ctx.angular = m.linkAngularVelocity(link), ctx.mass = mass, ctx.gravity = m.options().gravity;
                    V3d F{0, 0, 0}, T{0, 0, 0};
                    fn(ctx, F, T);
                    m.applyWrench(link, f.local ? rotate(p, F) : F, f.local ? rotate(p, T) : T, ctx.point);
                });
                break;
            }
            auto exprs = std::make_shared<std::array<ForceExpression, 6>>();
            std::string err;
            bool ok = true;
            for (std::size_t e = 0; e < 6 && ok; ++e)
                if (!f.script[e].empty()) ok = (*exprs)[e].compile(f.script[e], &err);
            if (!ok) {
                notes.push_back("force " + f.name + ": " + err);
                break;
            }
            const double mass = rig.links[static_cast<std::size_t>(link)].mass;
            mb.addForceHook([f, link, at, exprs, mass](Multibody& m) {
                if (!activeAt(f, m.time())) return;
                const LinkPose p = m.linkPose(link);
                const V3d pt = p.apply(at), v = pointVelocity(m, link, pt), w = m.linkAngularVelocity(link);
                const double vars[] = {m.time(), pt[0], pt[1], pt[2], v[0], v[1], v[2], w[0], w[1], w[2], mass, norm3(m.options().gravity)};
                double out[6];
                for (std::size_t e = 0; e < 6; ++e) out[e] = (*exprs)[e].empty() ? 0.0 : (*exprs)[e].eval(vars);
                const V3d F{out[0], out[1], out[2]}, T{out[3], out[4], out[5]};
                m.applyWrench(link, f.local ? rotate(p, F) : F, f.local ? rotate(p, T) : T, pt);
            });
            break;
        }
        }
    }
    return notes;
}

std::string describeForce(const ForceSpec& f) {
    char buf[400];
    std::string when;
    if (f.from > 0 || f.until >= 0) {
        std::snprintf(buf, sizeof buf, " from %g s", f.from);
        when = buf;
        if (f.until >= 0) std::snprintf(buf, sizeof buf, " until %g s", f.until), when += buf;
    }
    switch (f.kind) {
    case ForceKind::Wrench:
        std::snprintf(buf, sizeof buf, "%s: wrench on %s at (%g, %g, %g) mm: force (%g, %g, %g) N, torque (%g, %g, %g) N m%s", f.name.c_str(), f.body.c_str(), f.at.x, f.at.y,
                      f.at.z, f.force.x, f.force.y, f.force.z, f.torque.x, f.torque.y, f.torque.z, f.local ? " in its axes" : "");
        break;
    case ForceKind::Buoyancy:
        std::snprintf(buf, sizeof buf, "%s: buoyancy on %s, fluid %g kg/m^3 below %g mm, drag %g N s/m", f.name.c_str(), f.body.c_str(), f.density, f.level, f.drag);
        break;
    case ForceKind::Magnet:
        std::snprintf(buf, sizeof buf, "%s: magnet on %s, %g N within %g mm of %s", f.name.c_str(), f.body.c_str(), f.strength, f.range,
                      f.onto.empty() ? "the ground" : f.onto.c_str());
        break;
    case ForceKind::Script: {
        std::string e;
        if (!f.function.empty()) e = "C++ " + f.function;
        static const char* const names[] = {"fx", "fy", "fz", "tx", "ty", "tz"};
        for (std::size_t k = 0; k < 6 && f.function.empty(); ++k)
            if (!f.script[k].empty()) e += (e.empty() ? "" : ", ") + std::string(names[k]) + " = " + f.script[k];
        std::snprintf(buf, sizeof buf, "%s: script on %s at (%g, %g, %g) mm: %s%s", f.name.c_str(), f.body.c_str(), f.at.x, f.at.y, f.at.z, e.empty() ? "nothing" : e.c_str(),
                      f.local ? " (its axes)" : "");
        break;
    }
    }
    return std::string(buf) + when;
}

} // namespace sicdm

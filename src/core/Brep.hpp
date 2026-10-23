#pragma once

// An exact boundary representation (B-rep): the solid's faces lie on exact surfaces
// and its edges on exact curves, in double precision. It is what STEP files carry, and
// what the native primitives, extrusions and revolves make; the display mesh is
// tessellated from it to a tolerance.
//
//   Surfaces  plane, cylinder, cone, sphere, torus, B-spline (rational or not),
//             linear extrusion and revolution of a curve
//   Curves    line, circle, ellipse, B-spline (rational or not), polyline
//   Topology  vertices, edges on curves, loops of oriented edges (or a single vertex:
//             a pole or apex), faces on surfaces with their loops, shells (closed
//             solids or open surface models)

#include "core/Geometry.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sicdm {

struct DVec3 {
    double x = 0, y = 0, z = 0;
    DVec3() = default;
    constexpr DVec3(double a, double b, double c) : x(a), y(b), z(c) {}
    explicit DVec3(const Vec3& v) : x(v.x), y(v.y), z(v.z) {}
    Vec3 f() const { return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)}; }
    DVec3 operator+(const DVec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    DVec3 operator-(const DVec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    DVec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    DVec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    DVec3 operator-() const { return {-x, -y, -z}; }
    DVec3& operator+=(const DVec3& o) { return *this = *this + o; }
};
inline double ddot(const DVec3& a, const DVec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline DVec3 dcross(const DVec3& a, const DVec3& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double dlength(const DVec3& a) { return std::sqrt(ddot(a, a)); }
inline DVec3 dnormalize(const DVec3& a) {
    const double l = dlength(a);
    return l > 0 ? a / l : DVec3{0, 0, 0};
}

// A right-handed frame: origin and unit axes.
struct Frame {
    DVec3 o{0, 0, 0}, x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};
    // From an origin, a z axis and a reference direction for x (made square to z).
    static Frame make(const DVec3& o, const DVec3& z, const DVec3& xRef);
    DVec3 at(double a, double b, double c) const { return o + x * a + y * b + z * c; }
};

// A rigid placement (with a uniform scale): p' = o + s R p.
struct Placement {
    double r[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // row major
    DVec3 t{0, 0, 0};
    double s = 1.0;
    DVec3 point(const DVec3& p) const;
    DVec3 dir(const DVec3& d) const; // rotation only
    Placement then(const Placement& outer) const; // this, then outer
    static Placement fromFrames(const Frame& from, const Frame& to); // maps `from` onto `to`
    Placement inverse() const;
};

// The value a float dimension was typed as: the shortest decimal that reads back as `f`,
// in double (0.1f holds 0.100000001490116; this gives 0.1). Dimensions kept in floats go
// into exact geometry through this, so a STEP file says 0.1.
double decimal(float f);
inline DVec3 decimal(const Vec3& v) { return {decimal(v.x), decimal(v.y), decimal(v.z)}; }
// The rotation eulerRotation() makes (degrees about x, then y, then z), in double; quarter
// turns are exact.
Placement eulerPlacement(const DVec3& degrees);
// The other way: the degrees about x, then y, then z of a placement's rotation, to a
// ten-millionth of a degree (so quarter turns come out exact).
DVec3 eulerDegrees(const Placement& p);

struct BSplineCurve {
    int degree = 1;
    std::vector<DVec3> cp;
    std::vector<double> w;     // weights, or empty (not rational)
    std::vector<double> knots; // full: cp.size() + degree + 1
    DVec3 eval(double t) const; // a closed curve wraps outside its range
    bool closed() const;        // its ends meet
    double t0() const { return knots[static_cast<std::size_t>(degree)]; }
    double t1() const { return knots[knots.size() - 1 - static_cast<std::size_t>(degree)]; }
    mutable signed char m_closed = -1;
};

struct BSplineSurface {
    int du = 1, dv = 1;
    int nu = 0, nv = 0;        // control points: cp[i * nv + j], i along u
    std::vector<DVec3> cp;
    std::vector<double> w;     // or empty
    std::vector<double> ku, kv;
    DVec3 eval(double u, double v) const; // closed directions wrap outside their range
    bool closedU() const; // the first and last rows of control points meet
    bool closedV() const;
    double u0() const { return ku[static_cast<std::size_t>(du)]; }
    double u1() const { return ku[ku.size() - 1 - static_cast<std::size_t>(du)]; }
    double v0() const { return kv[static_cast<std::size_t>(dv)]; }
    double v1() const { return kv[kv.size() - 1 - static_cast<std::size_t>(dv)]; }
    mutable signed char m_cu = -1, m_cv = -1; // cached closedness (-1: not known yet)
};

enum class BCurveKind { Line, Circle, Ellipse, BSpline, Polyline };

struct BCurve {
    BCurveKind kind = BCurveKind::Line;
    Frame f;                 // line: o + t x; circle / ellipse: centre and plane (z: normal)
    double r1 = 0, r2 = 0;   // circle radius; ellipse semi-axes along x and y
    BSplineCurve bs;
    std::vector<DVec3> pts;  // polyline
    DVec3 eval(double t) const;
    bool periodic() const { return kind == BCurveKind::Circle || kind == BCurveKind::Ellipse; }
    double param(const DVec3& p) const; // the parameter of the point on the curve nearest p
};

enum class BSurfaceKind { Plane, Cylinder, Cone, Sphere, Torus, BSpline, Extrusion, Revolution };

struct BSurface {
    BSurfaceKind kind = BSurfaceKind::Plane;
    Frame f;
    double r1 = 0;  // cylinder / cone / sphere radius; torus major radius
    double r2 = 0;  // cone semi-angle (radians); torus minor radius
    BSplineSurface bs;
    int curve = -1;   // extrusion / revolution: the swept curve (in Brep::curves)
    DVec3 dir{0, 0, 1}; // extrusion: the direction and length per unit v
    // Point at (u, v). Plane: u, v along x, y. Cylinder / cone: u angle about z, v along z.
    // Sphere: u longitude, v latitude. Torus: u about z, v round the tube. B-spline: its
    // own parameters. Extrusion: u the curve's, v along dir. Revolution: u the curve's,
    // v angle about the axis (f.o, f.z).
    DVec3 eval(double u, double v) const;
    DVec3 normal(double u, double v) const; // the surface's own normal (du x dv), unit
    // Periodic parameters (0: none) and the parameter box.
    double periodU() const;
    double periodV() const;
    void domain(double& u0, double& u1, double& v0, double& v1) const;
};

struct BrepVertex {
    DVec3 p;
};

struct BrepEdge {
    int v0 = -1, v1 = -1;
    int curve = -1;        // -1: the straight segment v0 -> v1
    bool sameSense = true; // the curve runs from v0 to v1
};

struct OrientedEdge {
    int edge = -1;
    bool forward = true;   // v0 -> v1
};

struct BLoop {
    std::vector<OrientedEdge> edges; // or empty with vertex >= 0: a single point (pole / apex)
    int vertex = -1;
    std::vector<DVec3> poly;         // or a polygon (faceted B-reps)
    bool sense = true;               // false: run backwards
    bool outer = false;
};

struct BFace {
    int surface = -1;
    bool sameSense = true; // the face's outward normal is the surface's
    std::vector<BLoop> loops;
};

struct BShell {
    std::vector<BFace> faces;
    bool closed = true;
    std::string name;
};

struct Brep {
    std::vector<BrepVertex> vertices;
    std::vector<BrepEdge> edges;
    std::vector<BCurve> curves;
    std::vector<BSurface> surfaces;
    std::vector<BShell> shells;
    std::string name;

    bool empty() const { return shells.empty(); }
    std::size_t faceCount() const;
    void transform(const Placement& p);
};

struct TessellateOptions {
    double chord = 0.02;      // mm: largest gap between the mesh and the surface
    double angle = 0.26;      // radians: largest turn between neighbouring facets (about 15 deg)
    double maxEdge = 0.0;     // mm: longest triangle edge (0: no limit)
};

struct TessellateReport {
    std::size_t faces = 0, failedFaces = 0;
    std::string firstError;
    std::vector<std::size_t> faceTriangles; // triangles each face made, faces in shell order
};

// A closed (where the B-rep is) triangle mesh of every shell, in the B-rep's units.
MeshData tessellate(const Brep& brep, const TessellateOptions& o = {}, TessellateReport* report = nullptr);

// The surface's own normal (du x dv) where it is nearest p.
DVec3 surfaceNormalNear(const Brep& b, const BSurface& s, const DVec3& p);
// Turns loops to run counter-clockwise round each face seen from outside (holes
// clockwise), as other CAD systems read them. Loops that wrap round a periodic surface
// are left as they are.
void orientLoops(Brep& b);

// A fingerprint of a mesh's triangles (exact geometry belongs with one mesh).
std::uint64_t meshKey(const MeshData& m);
// A rotation-and-uniform-scale matrix (and translation) as a placement.
Placement placementOf(const Mat4& m);

// Exact B-reps of the native shapes (mm, the body's own frame).
std::shared_ptr<Brep> brepBox(const DVec3& lo, const DVec3& hi);
std::shared_ptr<Brep> brepCylinder(const DVec3& base, double r, double h);
std::shared_ptr<Brep> brepCone(const DVec3& base, double r0, double r1, double h);
std::shared_ptr<Brep> brepSphere(const DVec3& c, double r);
std::shared_ptr<Brep> brepTorus(const DVec3& c, double R, double r);
// A closed profile of (a, r) points (a along the axis, r >= 0 out from it, in the direction
// `ref` at the start) turned about the axis through `axisPoint` along `axis` by `sweep`
// radians, from `ref` towards axis x ref; a whole turn or more closes it. Null when the
// profile is degenerate or crosses the axis.
std::shared_ptr<Brep> brepRevolve(const std::vector<std::array<double, 2>>& profile, const DVec3& axisPoint, const DVec3& axis, const DVec3& ref,
                                  double sweep);
// A circle profile (centre (a, r), radius rho < r) turned the same way: a torus, or part.
std::shared_ptr<Brep> brepRevolveCircle(double a, double r, double rho, const DVec3& axisPoint, const DVec3& axis, const DVec3& ref, double sweep);

} // namespace sicdm

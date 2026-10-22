# STEP and IGES files, and exact geometry

SI-CDM reads and writes STEP (ISO 10303-21) and IGES (5.3) files. The geometry inside is
exact: boundary representations (B-reps) whose faces lie on true planes, cylinders, cones,
spheres, tori and NURBS surfaces, in double precision. They are what other CAD systems
exchange.

```text
IMPORT bracket.step            (or File → Import STEP / IGES...)
IMPORT housing.igs
EXPORT model.step              (or File → Export STEP / IGES...)
EXPORT model.igs
```

The readers and writers are SI-CDM's own code. No outside geometry library is used.

## Reading

**What is read:**

| Kind | Entities |
| --- | --- |
| Solids | `MANIFOLD_SOLID_BREP`, `BREP_WITH_VOIDS`, `FACETED_BREP`, and surface models (`SHELL_BASED_SURFACE_MODEL`) |
| Surfaces | planes, cylinders, cones, spheres, tori, B-spline surfaces (rational or not), linear extrusions and revolutions of curves, rectangular-trimmed surfaces |
| Curves | lines, circles, ellipses, B-spline curves (rational or not), polylines, and the 3D curve of surface, seam and trimmed curves |
| Topology | vertices, edges, edge, vertex and polygon loops, faces with holes, open and closed shells |

**Units:**

- **Length:** SI units with any prefix (mm, cm, m...) and conversion-based units such as the
  inch or foot all become millimetres.
- **Angles:** cone angles in degrees or radians are both read correctly.

**Assemblies:**

- Every part instance is placed through its representation relationships, at any depth.
- A part used twice comes in as two bodies, each where the assembly puts it.
- Bodies take their products' names. A name used twice gets a number.

**Display:**

- **Mesh:** each solid becomes a body with a display mesh, tessellated from the exact
  geometry to a tolerance that suits its size: 0.02 mm on a 100 mm part.
- **Closed:** edges are sampled once and shared by both faces, so the mesh comes out
  watertight.
- **Periodic faces:** seams, poles (a sphere's, a cone's apex) and faces that wrap all the way
  round are all handled.

The exact geometry stays with the body, so `EXPORT .step` writes it again unchanged.

## Writing

`EXPORT path.step` writes every visible body as AP214 (automotive_design), in millimetres,
one product per body.

**Exact:**

- STEP imports keep their exact geometry, as do the box, cylinder, cone, sphere, torus,
  wedge, pyramid and prism primitives.
- `EXTRUDE` bodies are exact: a circle makes a true cylinder, a polygon a prism.
- `REVOLVE` bodies are exact, whole turns and partial ones. Each straight side of the
  profile turns into a plane, a cylinder or a cone; a circle profile turns into a torus.
  Partial turns are closed by flat ends.
- Turned (`ROTATE`) and scaled (`SCALE`) bodies stay exact.

**Planar facets:** every other mesh goes out as a B-rep of planar faces, with coplanar
neighbouring triangles merged into one face. Closed meshes give closed solids.

**When exact becomes faceted:** a body's exact geometry belongs to one mesh. Editing that
mesh (a sculpt stroke, a boolean, a remesh, an edge fillet) leaves the exact geometry
behind, and the body exports as facets from then on. The report says how many bodies went
exact and how many as facets:

```text
Exported 8 bodies to model.step (STEP AP214, mm): 6 exact, 2 as planar facets
```

Projects keep the exact geometry of imported and revolved bodies (saved with each one), so
it survives `SAVE` and `OPEN`. Primitives and extrusions rebuild theirs from their
parameters.

## Precision

- **Exact geometry is double precision:** points, axes, radii and placements, about 15
  significant digits.
- **Dimensions as typed:** sizes, sketch points and angles are kept as single-precision
  numbers. Exact geometry takes each one as the decimal that was typed, so a 0.1 mm wall
  is 0.1 in the STEP file, not 0.100000001490116.
- **Turns:** `ROTATE` turns exact geometry in double precision, and quarter turns are
  exact. Turning a body away and back returns it to within 10^-12 mm.
- **Display meshes and toolpaths** use single precision. That is 0.06 µm at 1 m from the
  origin, far below a machine's resolution.

## IGES

**What is read:**

| Kind | Entities |
| --- | --- |
| Solids | B-rep solids (186 manifold solid, 514 shell, 510 face, 508 loop, 504 edge and 502 vertex lists), with voids |
| Surfaces | trimmed surfaces (144, bounded by 142 curves on them), and untrimmed B-spline surfaces |
| Surface geometry | planes (108, 190), cylinders (192), cones (194), spheres (196), tori (198), B-spline surfaces (128), surfaces of revolution (120), tabulated cylinders (122) |
| Curves | lines (110), circular arcs (100), elliptical arcs (104), B-spline curves (126), polylines (106), composite curves (102) |
| Placement | transformation matrices (124), at any depth |
| Units | the global section's unit (inch, mm, m, foot...), to millimetres |

**Sewing.** Most IGES files are trimmed surfaces with no topology between them, so the
reader sews them:

- **Shared edges:** boundary curves that run between the same points along the same curve
  become one edge, used by both faces. Two descriptions of one curve (an ellipse as an
  arc in one face and a B-spline in the next) still meet.
- **Bodies:** every group of faces that closes up becomes a solid, its faces turned to
  face outward. What doesn't close comes in as one surface body.
- **Tolerance:** points closer than the file's resolution (or 1e-5 of the model's size)
  are one.

**Simpler surfaces.** A surface of revolution whose generatrix is a line or a circular
arc comes in as the plane, cylinder, cone, sphere or torus it is. IGES writers often send
those shapes that way.

**Names.** IGES has no part names: bodies are named after the file.

**Writing.** `EXPORT model.igs` writes every visible body as trimmed surfaces (144), the
form CAD systems read most widely, in millimetres:

- **Surfaces:**
  - planes;
  - cylinders, cones, spheres and tori, as surfaces of revolution;
  - B-spline surfaces, extrusions and revolutions.
- **Boundaries:**
  - each face is bounded by its edges in model space;
  - loops run the way IGES readers expect, counter-clockwise about each surface's own
    parameter normal;
  - each surface's seam lies on the face's own seam.
- **Faceted bodies:** meshes without exact geometry go out as planar faces, as in STEP.

## How it is checked

The tests read 20 reference files written by an independent STEP writer (OpenCascade, used
only to make the files): `tests/data/step`.

- **Shapes:**
  - plain solids, and NURBS-converted ones;
  - fillets, holes and counterbores;
  - lofts, revolves, ellipse sweeps and oblique cuts;
  - a pointed cone, a hemisphere;
  - a two-level assembly;
  - a part in inches.
- **Every one:**
  - comes in closed;
  - matches the exact volume to 0.1% at a 0.002 mm tolerance, and to 0.7% at 0.02 mm;
  - reads back identical after SI-CDM writes it.
- **Independently checked:** OpenCascade reads every file SI-CDM writes as a valid solid with
  the exact volume. That includes the native shapes: primitives, extrusions, and revolves
  (whole and partial turns, profiles touching the axis, circle profiles).

**IGES:** the same shapes, written by OpenCascade both as trimmed surfaces and as B-rep
solids, 40 files in `tests/data/iges`:

- **Every one:**
  - comes in closed (the trimmed surfaces sewn);
  - matches the exact volume to 0.1% at a 0.002 mm tolerance;
  - is in place: the inch file in millimetres, the assembly's three parts where they belong.
- **Written by SI-CDM:** each reads back at its volume. OpenCascade sews every one into a
  valid solid with the exact volume, to within 0.00004%.
- **Round trips:** OpenCascade's IGES, imported and written again as IGES or as STEP, gives
  valid solids with exact volumes.

## Not yet

- **Exact operations:** booleans, fillets and shells on the exact geometry. These work on the
  mesh, which then exports as facets.
- **Other content:** colours, layers, product metadata and PMI (dimensions and tolerances in
  the file).

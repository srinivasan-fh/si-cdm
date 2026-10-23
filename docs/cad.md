# Part features and the feature history

These features complete the solid modelling tools. They work on sketch curves and on bodies,
from *Design* in the menu, the command line, or the `cad` MCP tool.

| Kind | Features |
| --- | --- |
| Sketch constraints | `CONSTRAIN`, `DIM`, `CONSTRAINTS` and `CONSTRAINT EDIT` / `DELETE` |
| Solids from sketch curves | `EXTRUDE`, `REVOLVE`, `SWEEP` and `LOFT` |
| Copies of bodies | `PATTERN` and `MIRROR` |
| Edge rounds and bevels | `EDGES`, `FILLET` and `CHAMFER` |
| Analysis | `MASSPROPS` and `MEASURE` |

Every model command goes into a **feature history** that can be edited and regenerated.

![A revolved vase, a loft through three sections, a swept pipe, and a rounded block with a pattern of holes](cad-features.png)

The picture comes from [examples/cad-demo.txt](examples/cad-demo.txt) (`RUN docs/examples/cad-demo.txt`).

## Sketching

### Curves

```text
LINE 0,0 40,0            PLINE 0,0 40,0 40,20 CLOSE        RECT 0,0 40,20        CIRCLE 20,10 5
ARC 10,0 0,10 -10,0      (start, a point on it, end)       ARC CENTER 0,0 10,0 0,10
SPLINE 0,0 10,8 20,-3 30,5 [CLOSE]                         ELLIPSE 0,0 20,0 8    (centre, major axis end, minor radius)
SKETCHSLOT 0,0 30,0 5    (end centres, radius)             POLYGON 0,0 10 6 [CIRCUMSCRIBED] [ANGLE deg]
```

**Construction curves:** add `CONSTRUCTION` at the end of any of these to make a reference
curve. Construction curves:

- are dashed and grey;
- guide the sketch and can be dimensioned and constrained;
- are never part of a profile.

`CONSTRUCTION "Line 3" [ON|OFF]` switches a curve either way. The tool options bar has the same
switch for new curves.

**Splines:**

- A spline passes through every point it is given.
- It is a centripetal Catmull-Rom curve, so it has no loops or cusps between close points.
- `CLOSE` joins it into a loop.

**Polygons:**

- `POLYGON` makes a closed polyline with its first corner along +u.
- The radius reaches the corners. With `CIRCUMSCRIBED` the radius reaches the middle of the sides
  instead (a nut across flats).

**What extrudes:**

- closed polylines, rectangles, circles, ellipses, slots and closed splines;
- lines, arcs, open polylines and splines that meet end to end in a closed loop. Picking any one
  of them uses the whole loop, so a slot drawn as two lines and two arcs extrudes like a slot.

### Trim, extend and offset

```text
TRIM "Line 1" AT 10,0          the piece at 10,0, back to the curves crossing it
EXTEND "Line 2" [START|END|AT x,y]
OFFSET "Circle 3" 2 [SIDE x,y]  (a negative distance: inside / right)
```

**Trim:**

- Removes the piece of a curve between its nearest crossings on either side of the point. A
  crossing at an end doesn't count.
- A line becomes two lines, a circle an arc, a rectangle or closed polyline an open polyline, and
  an ellipse or spline a spline.
- A curve nothing crosses goes altogether.
- `TRIM body PLANE ...` still slices a body.

**Extend:**

- Carries a line's or open polyline's end straight on, or an arc's end round its circle, to the
  nearest curve it meets.

**Offset:**

| Curve | Offset copy |
| --- | --- |
| Line | Parallel |
| Circle, arc | Concentric |
| Rectangle, slot | Grown or shrunk |
| Polyline | Each side moved, the corners mitred |
| Ellipse, spline | A spline through points the distance away along the normal |

The side is the one `SIDE` names. Without it, a closed curve grows outwards, an arc moves away
from its centre, and an open curve moves to its left. A negative distance turns each of these
around.

**In the app:**

- *Trim* (T): click the piece to cut away.
- *Extend*: click near the end to carry on.
- *Offset*: click the curve on the side to offset to; the distance is in the options bar.

### Constraint status

The solver works out how free each curve still is.

- **Green:** fully constrained.
- **Red:** has a constraint that can't be met.
- **The usual colour:** can still move.

The status bar sums it up, for example "Sketch: 3 fully constrained, 2 free".

**Constraining arcs:**

- An arc is constrained like a circle: its radius, concentric, tangent, point on.
- `START` and `END` pick its ends, and `CENTRE` its centre.
- `DIM "Arc 1" 15` sets its radius.

### Files

Projects keep every kind. DXF export handles each kind as follows:

| Kind | DXF entities |
| --- | --- |
| Arc | `ARC` |
| Slot | Two `LINE`s and two `ARC`s |
| Spline, ellipse | Fine polylines (DXF R12 has neither) |

Construction curves export on their own layer.

DXF import reads `ARC` as an arc, `SPLINE` through its fit (or control) points, and `ELLIPSE` as an
ellipse.

## Sketch constraints and dimensions

```text
CONSTRAIN kind curve [START|END|CENTRE|POINT n|SIDE n] [curve [...]]
DIM [HORIZONTAL|VERTICAL|RADIUS|DIAMETER|ANGLE] curve [selector] [curve [selector]] [value]
CONSTRAINTS
CONSTRAINT EDIT D1 value      CONSTRAINT DELETE C2
```

Sketch curves can be tied together by constraints and sized by dimensions, as in a parametric
CAD sketcher. Each new constraint is solved at once: the curves move as little as they must.

| Constraint | Between |
| --- | --- |
| `COINCIDENT` | two points |
| `HORIZONTAL`, `VERTICAL` | a line or side, or two points |
| `PARALLEL`, `PERPENDICULAR` | two lines or sides |
| `TANGENT` | a line and a circle, or two circles (outside or inside, from where they are) |
| `EQUAL` | two lines' lengths, or two circles' radii |
| `CONCENTRIC` | two circles |
| `ON` | a point on a line or circle |
| `MIDPOINT` | a point at the middle of a line or side |
| `FIX` | a point, held where it is |

`DIM` sizes a line's length, the distance of two points or of a point from a line, a circle's
radius or diameter, or the angle between two lines. Without a value it keeps the size there is
now. It picks its kind from what it is given: two lines make an angle, a circle a radius.

**Selectors:**

- `START` / `END`: a line's ends.
- `CENTRE`: a circle's centre.
- `POINT n` / `CORNER n`: a polyline vertex or rectangle corner.
- `SIDE n`: a polyline segment or rectangle side.

A closed polyline stays closed.

**How it is solved:** damped least squares (Levenberg–Marquardt) on the constraints' errors. A
constraint that can't be met with the others is refused, and nothing moves. Afterwards the rank
of the constraints' Jacobian gives the **degrees of freedom** left. Every reply says either
"fully constrained" or "free in N ways". `CONSTRAINTS` lists them all with their names (C1, D2...),
values and whether each is met. In the 2D view they are labelled on the sketch, in red when not
met.

**Driving the model:** constraints and dimensions are history steps. `CONSTRAINT EDIT D5 80`
rewrites that dimension's step and regenerates the history. So a part extruded from the sketch
is rebuilt at the new size, and so is everything done to it afterwards.

```text
PLINE 0,0 48,2 50,31 -1,29 CLOSE
CONSTRAIN HORIZONTAL "Polyline 1" SIDE 1      CONSTRAIN VERTICAL "Polyline 1" SIDE 2
CONSTRAIN HORIZONTAL "Polyline 1" SIDE 3      CONSTRAIN VERTICAL "Polyline 1" SIDE 4
DIM "Polyline 1" SIDE 1 60      DIM "Polyline 1" SIDE 2 40      CONSTRAIN FIX "Polyline 1" START
EXTRUDE "Polyline 1" 10
CONSTRAINT EDIT D5 80           (the block is rebuilt 80 long)
```

The constraints are saved with the project.

## Revolve

```text
REVOLVE [curve] [ANGLE deg] [AXIS U | V | "sketch line" | x,y x,y] [KEEP]
```

REVOLVE turns a closed sketch curve (rectangle, circle, closed polyline) about an axis in its
plane.

- **Axis:** the plane's vertical axis by default, falling back to the horizontal one. `AXIS`
  can also name a sketch line, or give two points.
- **Angle:** 360° gives a closed solid of revolution. Less gives a solid capped at both ends.
- **Profile:** it may touch the axis, as a vase's does, but not cross it.

```text
PLANE FRONT
PLINE 0,0 20,0 23,8 17,28 11,42 15,58 0,58 CLOSE
REVOLVE "Polyline 1" AXIS V
```

## Sweep

```text
SWEEP [profile] ALONG path [KEEP]
```

SWEEP carries a closed profile along a path curve: a line, polyline, rectangle or circle.

- **Placement:** the profile is centred on its centroid and turned square to the path's start.
- **Corners:** the profile is carried round each corner by that corner's own rotation (parallel
  transport). Each corner is cut on the plane that halves its angle, so the section stays true
  through the bend.
- **Closed paths:** a rectangle or circle path closes the tube. Any twist left after going
  round is spread along the path.

## Loft

```text
LOFT "section" [AT d] "section" [AT d] ... [SMOOTH | RULED] [KEEP]
```

LOFT makes a solid through two or more closed sections in order.

- **Offsets:** `AT d` places a section `d` along its plane's normal. Sections can lie on
  different planes.
- **Matching:** each section is resampled evenly and wound the same way. It starts where it
  best matches the section before it, so the solid doesn't twist.
- **Shape:** `RULED` joins the sections with straight lines. `SMOOTH` (the default) runs a
  Catmull-Rom curve through them.
- **Ends:** both ends are capped.

## Pattern

```text
PATTERN [body] LINEAR n SPACING d [DIR X|Y|Z|x,y,z] [ROWS m SPACING2 d DIR2 x,y,z]
PATTERN [body] CIRCULAR n [ANGLE a] [AXIS X|Y|Z|x,y,z] [AT x,y,z]
        [UNION "target" | SUBTRACT "target"]
```

PATTERN makes copies of a body: `n` along a line, or `n` × `m` in rows, or `n` round an axis.

- **Circular:** a full turn spaces the copies evenly. A partial `ANGLE` puts the first and last
  copies at its ends.
- **Names:** copies are named *Body (2)*, *Body (3)*, and so on.
- **Joining:** `UNION` joins the copies, and the original, into a target. `SUBTRACT` cuts them
  from it. For example, three holes through a plate:

```text
CYLINDER 4 20 @-20,0,-5
PATTERN "Cylinder 2" LINEAR 3 SPACING 20 SUBTRACT "Plate"
```

## Mirror

```text
MIRROR [body] [PLANE XY|YZ|XZ] [AT x,y,z] [NORMAL x,y,z] [COPY | MOVE] [UNION]
```

- **Plane:** the YZ plane through the origin by default. `AT` gives a point on the plane.
- **Copy:** a mirrored copy is the default. `MOVE` flips the body itself.
- **UNION:** joins the copy to the body, which makes a half model whole.

## Fillet and chamfer on chosen edges

```text
EDGES [body]
FILLET [body] RADIUS r EDGES ALL|CONVEX|CONCAVE|VERTICAL|HORIZONTAL|TOP|BOTTOM|n...|NEAR x,y,z...
CHAMFER [body] DISTANCE d [EDGES ...]
```

`EDGES` lists a solid's sharp edges, numbered. Faces meeting at more than 25° make an edge. Each
edge is a chain followed along the mesh: straight, or round like a cylinder's rim. The list says
whether each one is outside (convex) or an inside corner (concave), and gives its length and ends.

`FILLET ... EDGES` and `CHAMFER` treat just the chosen edges:

- by number, from the list;
- by a filter;
- or by the edge nearest each point (`NEAR`).

They are exact. The section at each point of the edge comes from its two faces. A cutter (or, on
an inside corner, a filler) is swept along it and cut from (or joined to) the solid. Where three
rounded outside edges meet, the corner becomes a ball, as in a CAD system. A chamfer sets each
face back by `d`. Without `EDGES`, CHAMFER takes the outside edges.

On a 40 × 30 × 20 box with R3 on every edge, the volume is within 0.03% of the exact rounded box.

## Fillet (rolling ball on every edge)

```text
FILLET [body] RADIUS r [CONVEX | CONCAVE | BOTH] [VOXEL v]
```

FILLET rounds a closed solid's edges with a rolling ball of radius `r`:

- **CONVEX** (the default) rounds the outside edges: a morphological opening.
- **CONCAVE** fills the inside corners: a closing.
- **BOTH** does the opening, then the closing.

**How it works:**

1. The solid's signed distance is computed on a voxel grid. The voxel is `r / 4` by default,
   up to 8 million voxels. Distances are exact near the surface, and closest points are spread
   by fast sweeping in all eight directions.
2. The set of ball centres (the solid shrunk by `r`) gets its own distance field.
3. Near the surface, each closest point is made exact by projected steps that keep it `r`
   from every face, so it settles on the shrunk solid's edges and corners.
4. The surface is remeshed from the field.

**Accuracy:** on a 40 × 30 × 20 box with R3, the surface is within 0.006 mm of the exact
rounded box on average and 0.06 mm at worst. The volume is within 0.3% of the Steiner formula.

**Limits:**

- Edges that aren't rounded stay sharp to within a voxel.
- A part thinner than the ball can't hold it, so FILLET refuses rather than erase it.
- It rounds all the edges of the kind chosen. To round only some, use `FILLET ... EDGES`
  (above).

## Mass properties

```text
MASSPROPS [body] [DENSITY g/cm3 | MATERIAL name]
```

```text
Block 1 (steel, 7.85 g/cm3)
  volume 24.000 cm3, surface 52.00 cm2, mass 188.40 g
  centre of mass (100, 0, 10) mm, size 40 x 30 x 20 mm
  inertia about the centre of mass (g mm2): Ixx 2.041e+04  Iyy 3.14e+04  Izz 3.925e+04  Ixy 0  Iyz 0  Ixz 0
  principal moments (g mm2): 2.041e+04  3.14e+04  3.925e+04
```

- **Method:** volume, surface area, centre of mass and the full inertia tensor about the centre
  of mass come from exact volume integrals over the closed mesh (the divergence theorem, one
  triangle at a time). The principal moments come from Jacobi rotations of the tensor.
- **Materials:** steel, stainless, aluminium, brass, copper, cast iron, iron, titanium,
  magnesium, gold, silver, ABS, PLA, PETG, nylon, POM, acrylic, carbon, wood, glass, concrete,
  water.

## Measure

```text
MEASURE x,y,z x,y,z
MEASURE "body" "body"
```

- **Two points:** the distance between them and its components.
- **Two bodies:** the closest distance between them, and the two closest points. It checks each
  body's corners against the other's triangles, both ways.

## Feature history

Every model command that succeeds is recorded in order: sketches, solids, features, cuts,
sculpting, moves, selections and plane changes.

```text
HISTORY                         the steps
HISTORY EDIT n command          change a step, and rebuild
HISTORY INSERT n command        add a step before step n, and rebuild
HISTORY DELETE n                remove a step, and rebuild
HISTORY CLEAR                   forget the history (the model stays)
REGEN [FORCE]                   rebuild the model from the history
```

```text
> HISTORY
History: 4 steps (REGEN rebuilds the model from them)
  1. BOX 60 40 15 @0,0,0
  2. HOLE "Block 1" AT -20,0,15 DIAMETER 6 THROUGH
  3. SELECT Block 1
  4. FILLET RADIUS 2
> HISTORY EDIT 2 HOLE "Block 1" AT -20,0,15 DIAMETER 10 THROUGH
Changed step 2 and regenerated 4 steps
```

**Rebuilding:**

- REGEN starts from an empty model and replays each step. Each step uses the units it was
  typed in (MCP tools' millimetres included).
- If a step fails, for example a hole that now misses the part, the model and the history stay
  as they were. The report gives the failing step and why.

**Undo and saving:**

- `UNDO` takes the last step off the history, and `REDO` puts it back.
- The history is saved with the project.

**When REGEN would lose work:** the model may change outside the command line, for example by a
brush stroke, a drag or a panel edit. The history may also start on a model that already had
parts. In either case `HISTORY` says so, and `REGEN` refuses unless you write `REGEN FORCE`.

## Over MCP

The `cad` tool has these operations:

- `revolve`, `sweep`, `loft`;
- `constrain`, `dim`, `constraints`, `constraint_edit`, `constraint_delete`;
- `pattern`, `mirror`, `edges`, `fillet`, `chamfer`;
- `massprops`, `measure`;
- `history`, `regen`, `history_edit`, `history_insert`, `history_delete`.

```json
{"operation": "revolve", "curve": "Polyline 1", "axis": "v"}
{"operation": "pattern", "body": "Boss", "type": "circular", "count": 6, "axis": "z"}
{"operation": "fillet", "body": "Housing", "radius": 2, "edges": "both"}
{"operation": "fillet", "body": "Block 1", "radius": 3, "select": "top"}
{"operation": "chamfer", "body": "Block 1", "distance": 1, "select": "1 4 7"}
{"operation": "dim", "refs": [{"curve": "Polyline 1", "at": "side 1"}], "value": 60}
{"operation": "constraint_edit", "constraint": "D5", "value": 80}
{"operation": "massprops", "body": "Housing", "material": "aluminium"}
{"operation": "history_edit", "step": 2, "command": "HOLE \"Block 1\" AT -20,0,15 DIAMETER 10 THROUGH"}
```

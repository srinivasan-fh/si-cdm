# Part features and the feature history

These features complete the solid modelling tools. They work on sketch curves and on bodies,
from *Design* in the menu, the command line, or the `cad` MCP tool.

| Kind | Features |
| --- | --- |
| Sketch planes and references | `PLANE`: Top, Front, Right, and datum planes anywhere or on a face; `REFAXIS`, `REFPOINT` |
| Multibody | `SPLIT` a body in two; `BOOLEAN` joins, cuts and intersects bodies |
| Sketch constraints | `CONSTRAIN`, `DIM`, `CONSTRAINTS` and `CONSTRAINT EDIT` / `DELETE` |
| Solids from sketch curves | `EXTRUDE`, `REVOLVE`, `SWEEP` and `LOFT` |
| Holes and threads | `HOLE`: standard sizes, counterbored, countersunk and tapped; `THREAD`: modelled threads |
| Copies of bodies | `PATTERN` and `MIRROR` |
| Moulding | `DRAFT`: side faces tilted to come out of a mould |
| Sheet metal | `SHEETMETAL`: base plate, flanges, flat pattern |
| Edge rounds and bevels | `EDGES`, `FILLET` and `CHAMFER` |
| Analysis | `MASSPROPS`, `MEASURE` and `FEA` (stress analysis) |
| Design intent | `VAR`: global variables and equations; `CONFIG`: configurations and design tables |

Every model command goes into a **feature history** that can be edited and regenerated.

![A revolved vase, a loft through three sections, a swept pipe, and a rounded block with a pattern of holes](cad-features.png)

The picture comes from [examples/cad-demo.txt](examples/cad-demo.txt) (`RUN docs/examples/cad-demo.txt`).

## Sketching

### Sketch planes and datum planes

```text
PLANE TOP | FRONT | RIGHT | name                       the sketch plane
PLANE OFFSET d [FROM plane]                            parallel to a plane, d along its normal
PLANE ANGLE deg [ABOUT U | V] [FROM plane]             a plane turned about its own u or v axis
PLANE THROUGH x,y,z x,y,z x,y,z                        through three points
PLANE AT x,y,z NORMAL x,y,z [U x,y,z]                  through a point, square to a direction
PLANE FACE x,y,z [ON body]                             on the face of a body nearest the point
PLANE LIST | PLANE DELETE name
```

New curves go on the **sketch plane**. Besides the three main planes, any plane can be a sketch
plane: each of the forms above makes a **datum plane** (named "Plane 1", "Plane 2"... or with
`NAME`) and makes it the sketch plane. `FROM` starts from a main plane or a datum plane, by
default the sketch plane itself.

- A datum plane's u axis is level where it can be (so v points up), and along X on level
  planes. The main planes come out the same: `PLANE AT 0,0,0 NORMAL 0,-1,0` is Front.
- Sketches drawn on a datum plane stay on it. `EXTRUDE`, `REVOLVE` (about the plane's u or v
  axis, or a line on it) and `SWEEP` work on them as on the main planes, and circles extrude
  to exact cylinders along the plane's normal.
- `2D name` opens the 2D Drawing window square to a datum plane, u to the right. The options
  bar lists the datum planes beside Top, Front and Right.
- Datum planes are saved with the project, and the `PLANE` steps are in the feature history:
  REGEN makes them again (a `FACE` plane finds its face on the rebuilt bodies).
- DXF and SVG export and import work on a datum plane; images go on the main planes.

```text
BOX 60 40 15 @0,0,0
PLANE FACE 10,10,40 NAME Lid
CIRCLE 0,0 6
EXTRUDE 12
PLANE THROUGH 0,0,15 30,0,40 0,20,15
RECT 5,5 20,15
EXTRUDE 4
```

### Reference axes and points

```text
REFAXIS name THROUGH p p | AT p DIR x,y,z | PLANES plane plane | OF body
REFPOINT name AT x,y,z | CENTRE body
REFAXIS LIST | REFPOINT LIST | REFAXIS DELETE name | REFPOINT DELETE name
```

- **Axes:** through two points, through a point along a direction, where two planes meet,
  or a round body's own axis (`OF`: its axis of symmetry, through its centre of mass).
- **Points:** at a position, or a body's centre of mass.
- Planes take them: `PLANE AT P1 NORMAL A1`, `PLANE THROUGH P1 P2 P3`; so do `PATTERN ...
  CIRCULAR n AXIS A1` (about that line) and `REVOLVE ... AXIS A1` (an axis lying in the
  profile's plane).
- They are drawn in the 3D window with their names (datum planes as framed squares, the
  sketch plane's highlighted), saved with the project, and REGEN makes them again.

### Splitting a body

```text
SPLIT [body] BY plane              TOP, FRONT, RIGHT or a datum plane
SPLIT [body] [PLANE] x,y,z NORMAL x,y,z
```

SPLIT cuts a body in two along a plane and keeps both pieces, the cuts capped: the piece the
normal points to becomes *Body (2)*. Hole wizard callouts stay with the piece they're in.

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

### Fillets, chamfers, mirrors and patterns

```text
SKETCHFILLET curve [CORNER n...] radius      every corner of a polyline or rectangle, or those named
SKETCHFILLET line line radius                where two lines meet (or would, extended)
SKETCHCHAMFER ...                            the same, bevelled by a distance
SKETCHMIRROR curve... ABOUT line | ABOUT x,y x,y
SKETCHPATTERN curve... LINEAR n dx,dy [ROWS m dx,dy] | CIRCULAR n [ABOUT x,y] [ANGLE deg]
```

- **Fillet / chamfer:** a rounded rectangle or polyline becomes lines and arcs end to end,
  which still extrude as one profile. Corners count from 1 along the curve (a rectangle's
  counter-clockwise from its first corner). Two lines are trimmed (or extended) to the
  arc. A fillet too big for a side is refused.
- **Mirror:** copies across a line (a construction centreline, say) or through two points.
  Arcs stay counter-clockwise; a rectangle mirrored across a slanted line becomes a closed
  polyline.
- **Pattern:** `LINEAR` n copies (the original the first) `dx,dy` apart, in `ROWS` too;
  `CIRCULAR` n round a point, evenly round a full turn, or spread over `ANGLE`.

### Sketching from bodies

```text
PROJECT body [EDGES | SECTION] [CONSTRUCTION]
```

- **EDGES:** the body's edges seen square to the sketch plane, as lines and polylines
  (edges seen end on, or lying on others, once).
- **SECTION:** where the sketch plane cuts the body, as closed polylines: profiles to
  extrude, revolve or cut from.
- `CONSTRUCTION` makes them reference curves.

### Dragging

With the **Select** tool, drag sketch geometry and the constraints hold while it moves:

- **A point:** an end, corner, vertex or centre. A small square marks each point of the selected
  curve, and the one under the cursor lights up.
- **A circle's or arc's rim:** changes its radius.
- **Anything else on a curve:** moves the whole curve without changing its shape.

The curves joined to it by constraints follow, moving as little as they must; curves not
connected to it stay put. When the constraints stop the grip short of the cursor, it goes as far as
they let it, and the status bar says how far short ("held 5.8 mm short by its constraints"). A
dimension or a fixed point holds firm. Esc during a drag puts everything back.

Each drag is recorded as a `DRAG` command, so it is in the history and undoes:

```text
DRAG "Polyline 1" POINT 3 TO 60,50      a point (START, END, CENTRE, POINT n, CORNER n)
DRAG "Circle 2" RIM TO 45,20            the radius, through 45,20
DRAG "Line 1" BY 5,0                    a whole curve, moved by 5,0
DRAG "Line 1" TO 10,10 FROM 0,0         a whole curve, the point 0,0 on it going to 10,10
```

**Undo:** every sketch edit undoes and redoes, with Ctrl+Z and Ctrl+Shift+Z (or Ctrl+Y; Cmd on
macOS), or `UNDO` / `REDO` and the File menu:

- new curves;
- trims, extends and offsets;
- construction changes;
- constraints and dimensions;
- drags.

The same UNDO and REDO cover the rest of the model, one command at a time, in the order the
changes were made:

- bodies (made, cut, moved, sculpted) and their parameters (`PARAM`, `REGEN`);
- mates, together with the move each one made;
- the drawing sheet;
- variables, configurations, datum planes, reference axes and points;
- motions, poses and physics (a body's and the world's settings);
- robot rigs and terrain;
- CAM and lathe jobs.

Undoing a change that isn't a history step (the sheet, a CAM job) leaves the feature
history as it is.

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

## Extrude

```text
EXTRUDE [curve] distance [SYMMETRIC | TWO d2] [REVERSE] [DRAFT deg] [THIN t] [CUT | JOIN [body]] [KEEP]
EXTRUDE [curve] THROUGH [SYMMETRIC] [REVERSE] ...          through every body (or the CUT body)
EXTRUDE [curve] UPTO NEXT | body | x,y,z [REVERSE] ...     up to the next face, a body, a point's level
```

EXTRUDE pushes a closed sketch curve (or a loop of curves end to end) along its plane's
normal into a solid.

- **Depth:** `distance` along the normal (negative or `REVERSE`: the other way), `SYMMETRIC`
  half each way, or `TWO d2`: `distance` one way and `d2` back the other.
- **THROUGH:** past every visible body along the normal (both ways with `SYMMETRIC`), the
  usual end for a hole.
- **UPTO:** `NEXT` stops where the whole profile has met a face ahead of it; a body's name
  stops at that body; a point stops at the plane through it, square to the normal.
- **DRAFT deg:** the sides lean in by that angle going away from the sketch plane (out when
  negative); both ways from the plane with `SYMMETRIC` or `TWO`. Too steep for the depth and
  it is refused.
- **THIN t:** a wall `t` thick inside the outline instead of the whole area. An open curve
  (a line, polyline, arc or spline) makes a rib `t` thick, centred on it.
- **CUT / JOIN:** cuts the solid from the bodies it meets, or joins them into one (`CUT body`,
  `JOIN body`: just that one). A cut that meets nothing is refused and nothing changes.

A plain extrusion stays parametric, with exact circles; drafts and walls from straight-sided
profiles are exact too (flat faces).

```text
BOX 60 40 20 @0,0,0
PLANE FACE 0,0,30 NAME Lid
CIRCLE 0,0 5
EXTRUDE THROUGH REVERSE CUT        a hole through the block
RECT -25,-15 -15,-5
EXTRUDE 8 DRAFT 5 JOIN             a tapered boss on the lid
PLINE -20,10 20,10
EXTRUDE 6 THIN 2 JOIN              a rib
```

## Holes

```text
HOLE [body] [AT] x,y,z [x,y,z...] [DIR x,y,z] SIZE size [FIT CLOSE | NORMAL | LOOSE]
     [DEPTH h | THROUGH] [CBORE [d depth] | CSINK [d [deg]]] [TAPPED [THREAD depth]] [FLAT]
HOLE [body] [AT] x,y,z [DIR x,y,z] DIAMETER d [DEPTH h | THROUGH]
HOLE SIZES                      the standard sizes and what goes with them
HOLE LIST [body]                the holes made, with their callouts
```

The hole wizard drills holes at points on a body's surface, square to it (or along `DIR`),
one or many at once.

- **SIZE:** ISO metric coarse, M1.6 to M24, or UNC, #2-56 to 1-8 (`M6`, `M6x1`, `1/4-20`,
  `10-24`). A plain hole is that size's clearance hole (ISO 273): `FIT CLOSE`, `NORMAL` (the
  default) or `LOOSE`. `DIAMETER` overrides the drill.
- **CBORE:** a counterbore for a socket head cap screw of that size (ISO 4762), or the
  diameter and depth given. **CSINK:** a countersink for a flat head screw (90 degrees
  metric, 82 inch), or the diameter and angle given.
- **TAPPED:** the tap drill, with the thread as a cosmetic thread in the hole's callout.
  A blind tapped hole's thread stops two pitches short of the drill depth unless `THREAD`
  says how deep.
- **DEPTH** is the drilled depth from the surface; blind holes end in a 118 degree drill
  point (`FLAT`: a flat bottom).
- Each hole keeps its callout on the body, saved with the project: `M8 tapped 12.5 deep
  (drill 6.8 x 15)`, `dia 6.6 thru (M6 clearance), counterbore 11 x 6.4`.

```text
BOX 80 60 20 @0,0,0
HOLE "Block 1" AT -25,-15,20 25,-15,20 SIZE M6 CBORE THROUGH
HOLE "Block 1" AT -25,15,20 25,15,20 SIZE M8 TAPPED DEPTH 15
HOLE "Block 1" AT 0,0,20 SIZE 1/4-20 CSINK THROUGH
```

## Threads

```text
THREAD [body] AT x,y,z AXIS x,y,z SIZE M10 | DIAMETER d PITCH p  LENGTH l [INTERNAL] [LEFT]
THREAD [body] HOLES | HOLE n
```

THREAD cuts a modelled ISO metric thread (60 degree flanks, the basic profile) into a body:
on a rod `LENGTH` long from `AT` along `AXIS` (the rod's diameter is the thread's), or with
`INTERNAL` into the wall of a hole drilled at the minor diameter. `HOLES` threads the
body's tapped hole wizard holes (each once; `HOLE n`: just that one, numbered as `HOLE
LIST` shows them). `LEFT` makes a left-hand thread. A tapped hole is otherwise a cosmetic
thread: the tap drill and its callout.

```text
CYLINDER 5 30
THREAD "Cylinder 1" AT 0,0,30 AXIS 0,0,-1 SIZE M10 LENGTH 20
```

## Revolve

```text
REVOLVE [curve] [ANGLE deg] [AXIS U | V | "sketch line" | x,y x,y] [CUT | JOIN [body]] [KEEP]
```

REVOLVE turns a closed sketch curve (rectangle, circle, closed polyline) about an axis in its
plane.

- **Axis:** the plane's vertical axis by default, falling back to the horizontal one. `AXIS`
  can also name a sketch line, or give two points.
- **Angle:** 360° gives a closed solid of revolution. Less gives a solid capped at both ends.
- **Profile:** it may touch the axis, as a vase's does, but not cross it.
- **CUT / JOIN:** cuts the revolved solid from the bodies it meets (a groove), or joins it on.

```text
PLANE FRONT
PLINE 0,0 20,0 23,8 17,28 11,42 15,58 0,58 CLOSE
REVOLVE "Polyline 1" AXIS V
```

## Sweep

```text
SWEEP [profile] ALONG path [KEEP]
SWEEP [profile] HELIX PITCH p TURNS n [AXIS U | V | x,y x,y] [LEFT] [KEEP]
```

SWEEP carries a closed profile along a path curve: a line, polyline, rectangle or circle.
With `HELIX PITCH p TURNS n` it goes round a helix instead (springs, coils): about the
profile plane's vertical axis by default (`AXIS U`, `V` or two points `x,y x,y` in that
plane), from the profile, rising `p` a turn (`LEFT`: left-handed). The profile must stay off
the axis and the coils clear of each other.

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
PATTERN [body] ALONG curve n [ALIGN]
PATTERN [body] POINTS [REF x,y,z]
PATTERN [body] FILL curve SPACING d [SPACING2 d] [STAGGER] [MARGIN m] [REF x,y,z]
        [UNION "target" | SUBTRACT "target"]
```

PATTERN makes copies of a body: `n` along a line, or `n` × `m` in rows, or `n` round an axis.

- **Along a curve:** `n` spaced evenly along a sketch curve, the original at its start
  (round a closed one, evenly all the way). `ALIGN` turns each copy with the curve.
- **At sketch points:** a copy at each visible sketch point (`POINT x,y`). The body's
  reference point goes onto each: its centre, on the points' plane, or `REF`.
- **Filling a region:** copies on a grid `SPACING` apart (rows `SPACING2` apart, the same by
  default) inside a closed sketch profile, `MARGIN` in from its edge; `STAGGER` shifts every
  other row by half. The grid runs through the reference point (as for `POINTS`), where the
  original stays.

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

## Sheet metal

```text
SHEETMETAL BASE curve THICKNESS t [RADIUS r] [K k]     a base plate from a closed profile
SHEETMETAL EDGES [body]                                the edges a flange can go on
SHEETMETAL FLANGE [body] EDGE n | NEAR x,y,z LENGTH l [ANGLE deg] [DOWN] [RADIUS r] [INSET a b]
SHEETMETAL MITER [body] EDGES n,m,... LENGTH l [ANGLE deg] [GAP g] [DOWN]   flanges round corners
SHEETMETAL HEM [body] EDGE n | NEAR x,y,z LENGTH l [OPEN gap | CLOSED] [DOWN]
SHEETMETAL JOG [body] EDGE n | NEAR x,y,z OFFSET h LENGTH l [ANGLE deg] [DOWN]
SHEETMETAL RELIEF [body] [SIZE s]                      corner reliefs where flanges meet
SHEETMETAL CONVERT body FACE x,y,z [RADIUS r] [K k]    a plate or shelled box as sheet metal
SHEETMETAL FLAT [body] [SKETCH]                        the flat pattern
SHEETMETAL INFO [body]
```

A sheet metal part is one sheet, `t` thick, bent along straight lines. It starts as a base
plate (any closed profile, on its plane) and grows flanges: strips on a free edge of the
base, or on a flange's far edge, bent through `ANGLE` (90 by default, up to 180; `DOWN`
bends the other way) with an inside radius (`RADIUS`, the part's by default, which is `t`
unless given). `LENGTH` is the flat beyond the bend; the bend is added outside the edge.

- **Bends:** the sheet follows the arc in each bend, its neutral layer `K t` from the inside
  (`K` 0.44 by default) keeping its length, so a bend's flat length is the bend allowance
  `BA = angle x (r + K t)`. `SHEETMETAL INFO` lists each bend's allowance and the flat size.
- **Flat pattern:** the part is kept as its flat pattern folded, so the two always agree.
  `FLAT` makes the flat plate as a body beside the part; with `SKETCH` also its outline
  (one closed polyline) and the bend lines (construction lines) as sketch curves, ready for
  `EXPORT flat.dxf`.
- Edges are numbered as `SHEETMETAL EDGES` lists them (each takes one flange); `NEAR x,y,z`
  picks the one nearest a point. The part is parametric: saved, undone and rebuilt as such.

**More flanges:**

- **INSET a b** holds a flange back from its edge's ends, by `a` at the edge's start and `b`
  at its end.
- **MITER** puts flanges on several base edges at once. Where two of them meet at a corner,
  each is cut on the corner's bisector: across the bend zone (so the bends don't run into
  each other) and, for walls that lean out (`ANGLE` under 90), along the wall too. The
  corners then close, `GAP` apart (a tenth of the sheet by default).
- **HEM** folds an edge back flat on itself (180 degrees). **OPEN gap** leaves that gap
  between the sheets (the inside radius is half of it). **CLOSED**, the default, leaves
  none.
- **JOG** makes two bends the opposite way, so the next flat steps `OFFSET` away (mid-sheet
  to mid-sheet) and runs on `LENGTH`. The flat between the bends is worked out exactly.
  A jog smaller than its two bends' rise is refused.
- **RELIEF** holds back the flanges at each corner where two of them meet (the bend radius
  plus the thickness by default, or `SIZE`). This is a rectangular relief, so the bends
  clear each other.
- **CONVERT** turns a thin-walled solid into sheet metal: a plate, or a box hollowed by
  `SHELL ... FACE`. Pick the outside of the base with `FACE`:
  - The thickness is measured under it.
  - Walls standing on its edges become 90-degree flanges, `r + t` shorter, so the part
    keeps its outside size.
  - Where two walls meet, the corner is mitered shut.
  - The body becomes a parametric sheet metal part with a flat pattern.

```text
RECT 0,0 80,50
SHEETMETAL BASE "Rectangle 1" THICKNESS 1.5 RADIUS 1.5
SHEETMETAL MITER "Sheet 1" EDGES 1,2,3 LENGTH 20     a tray: three walls, mitered corners
SHEETMETAL HEM "Sheet 1" EDGE 1 LENGTH 8 OPEN 1      a rolled front edge

BOX 60 40 20
SHELL "Block 2" THICKNESS 1.5 FACE 0,0,20
SHEETMETAL CONVERT "Block 2" FACE 0,0,0 RADIUS 1.5   a box that can be bent from a flat blank
```

```text
RECT 0,0 100,60
SHEETMETAL BASE "Rectangle 1" THICKNESS 2 RADIUS 1
SHEETMETAL FLANGE "Sheet 1" EDGE 1 LENGTH 20
SHEETMETAL FLANGE "Sheet 1" NEAR 0,30,0 LENGTH 10 DOWN
SHEETMETAL FLAT "Sheet 1" SKETCH
```

## Draft

```text
DRAFT [body] ANGLE deg [PULL x,y,z | X | Y | Z] [NEUTRAL x,y,z]
```

DRAFT tilts the faces that run along the pull direction (+Z by default), the sides a part
slides out of a mould along, by the angle: they lean in going along the pull (out with a
negative angle). They turn about the neutral plane, through `NEUTRAL` square to the pull,
by default the body's bottom (it keeps its size there). Corners stay mitred, so a block
becomes a frustum and a cylinder a cone. A draft that would fold faces over is refused.

```text
BOX 40 40 20 @0,0,0
DRAFT "Block 1" ANGLE 5
```

## Fillet and chamfer on chosen edges

```text
EDGES [body]
FILLET [body] RADIUS r EDGES ALL|CONVEX|CONCAVE|VERTICAL|HORIZONTAL|TOP|BOTTOM|n...|NEAR x,y,z...
FILLET [body] RADIUS r TO r2 EDGES ... [START x,y,z]      a variable fillet
FILLET [body] FULLROUND x,y,z                             a full round on the face at the point
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


**Variable fillets:** `RADIUS r TO r2` runs the radius from `r` to `r2` along each chosen
edge, from its end nearest `START` (its first end without one). Open edges only.

**Full rounds:** `FULLROUND x,y,z` rounds the face at the point, between two outside edges,
into one half circle: both edges at half the face's width.

```text
BOX 60 6 30
FILLET "Block 1" FULLROUND 0,0,30
```

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

## Shell, dome, rib and lettering

```text
SHELL   [body] THICKNESS t [OUTWARD] FACE x,y,z            hollow the part, open at that face
DOME    [body] FACE x,y,z HEIGHT h [CONCAVE]                raise (or dish in) a flat face
RIB     [curve] THICKNESS t [BODY name] [FLIP] [KEEP]       a wall from an open curve to the body
EMBOSS  [body] TEXT "words" FACE x,y,z HEIGHT h DEPTH d [ANGLE deg] [FONT sans|bold|mono]
ENGRAVE [body] TEXT "words" FACE x,y,z HEIGHT h DEPTH d [ANGLE deg] [FONT sans|bold|mono]
```

**SHELL ... FACE** hollows the part with walls `t` thick and leaves the face at the point
open, the way a CAD shell removes a chosen face. (`OPEN x,y,z NORMAL x,y,z` cuts the opening
along any plane instead.)

**DOME** raises the flat face at the point. Its outline shrinks inward by `R (1 - cos a)` as it
rises by `h sin a`, so it meets the face square and is flat on top. `R` is how far the outline
can shrink before it turns over. On a round face this is half an ellipsoid. **CONCAVE** dishes
the same shape into the face.

**RIB** grows a wall `t` thick, centred on the sketch plane, from an open curve to the body.
The wall runs square to the curve's chord, toward the body (**FLIP** for the other side). It
follows the surface under each point of the curve and reaches a little into it. The body is
the nearest one that way, unless you name one with `BODY`. The curve is hidden unless you add
**KEEP**.

**EMBOSS** raises lettering on the face at the point, and **ENGRAVE** cuts it in:

- **HEIGHT** is the height of the capitals and **DEPTH** how far the letters stand proud or go in.
- The words are centred on the point. They read along +X on a top face and upright on a side
  face, and **ANGLE** turns them. `\n` starts a new line.
- The letters come from the built-in fonts, read from their TrueType outlines: **bold** (Inter
  SemiBold, the default), **sans** (Inter) and **mono** (JetBrains Mono). These cover Latin
  letters, digits and the usual symbols.
- On a curved face the outlines are cut finer, and every point follows the surface along the
  face's normal. The letters then stand an even `DEPTH` proud of a cylinder or a sphere.
- Text that runs off the face is refused.

```text
BOX 80 30 10 @0,100,0
EMBOSS "Block 1" TEXT "SI CDM" FACE 0,100,10 HEIGHT 8 DEPTH 1
ENGRAVE "Block 1" TEXT "Part 42-B" FACE 0,85,5 HEIGHT 5 DEPTH 0.5

BOX 60 40 5
BOX 5 40 40 @-27.5,0,0
BOOLEAN "Block 2" UNION "Block 3"
PLANE FRONT
LINE -25,25 15,5
RIB THICKNESS 4                                            a 40 x 20 gusset in the corner
```

## Surfaces

Surfaces are open bodies with no thickness. Each one lies on an exact B-spline surface,
which is kept with the body, exported to STEP as a surface model and saved in the project.

```text
SURFACE LOFT curve curve [curve ...]                     through sections, in order
SURFACE BOUNDARY c1 c2 c3 c4                             filling four curves that meet end to end
SURFACE EXTRUDE curve DISTANCE d [DIR x,y,z]             an open (or closed) curve pulled straight
SURFACE REVOLVE curve AXIS x,y,z [AT x,y,z] [ANGLE deg]  a curve turned about an axis
SURFACE PLANAR curve                                     a flat surface filling a closed curve
SURFACE OFFSET body DISTANCE d                           a surface d away along its normals
SURFACE BLEND bodyA bodyB [WEIGHT w]                     a bridge between two surfaces' facing edges
THICKEN body THICKNESS t [BOTH]                          a solid from a surface
KNIT body body ... [TOLERANCE t]                         surfaces stitched into one (a solid when closed)
ANALYZE ZEBRA [body]                                     reflection stripes
ANALYZE CURVATURE [body] [MEAN|GAUSS]                    curvature in colour
ANALYZE OFF                                              back to normal
```

**How they are built:**

- Each surface interpolates its curves with cubic B-splines (degree 1 across two sections). The
  parameters follow chord length and the knots are averaged, so the surface passes through
  every section and point it is given.
- **LOFT** lines up closed sections at their nearest points and runs them the same way round.
  Sections must be all closed or all open.
- **BOUNDARY** takes the four curves in any order and either direction, and fills them with a
  bilinearly blended (Coons) patch. The patch has their exact edges.
- **BLEND** finds the edges of the two surfaces that face each other. It leaves each surface
  along that surface's own slope (a cubic Hermite bridge), so it meets both smoothly.
  **WEIGHT** sets how far the slope carries (default 1).

**Solids from surfaces:**

- **THICKEN** offsets the surface along its normals: one way, the other with a negative `t`,
  or both ways with **BOTH**. It walls the open edges.
- **KNIT** joins surfaces along edges they share. Points within the tolerance become one, and
  an edge with points the other side lacks is split there. The default tolerance is
  0.02 mm, or more on large parts. A tube and its two caps knit into a closed solid.

**Analysis:**

- **ZEBRA** paints stripes of the reflected view that move as you turn the model, like a
  striped light box. On a smooth surface the stripes run on smoothly. A kink or a jump in
  them shows a crease, or a change of curvature, where two surfaces meet.
- **CURVATURE** colours the surface blue to red by mean curvature (1/mm, signed), or by
  Gaussian curvature with **GAUSS**.
- Both draw on a copy finely tessellated from the exact surface and hide the original.
  **ANALYZE OFF** puts it back.

```text
CIRCLE 0,0 20
PLANE OFFSET 30
CIRCLE 0,0 10
PLANE OFFSET 30
CIRCLE 5,0 15
SURFACE LOFT "Circle 1" "Circle 2" "Circle 3"
ANALYZE ZEBRA "Surface 1"
```

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

## Analysis (FEA): stress, vibration, heat

```text
FEA [body] FIX FACE x,y,z | FIX NEAR x,y,z r ...
           LOAD FACE x,y,z | LOAD NEAR x,y,z r  FORCE fx,fy,fz ...
           PRESSURE FACE x,y,z MPa                 a pressure on a face (N/mm2)
           BEARING x,y,z r FORCE fx,fy,fz          a pin pushing on a hole
           GRAVITY [x,y,z]                         the body's own weight (in g; down by default)
           TEMP FACE x,y,z C   HEAT FACE x,y,z W   faces held at a temperature, heat flowing in
           THERMAL [REF C]                         stress from the heat's expansion
           MODES n   MODE k                        natural frequencies; plot mode k's shape
           [MATERIAL name] [E MPa] [NU v] [YIELD MPa] [CELL size] [PLOT] [DEFORM scale]
FEA MATERIALS
```

FEA works out how a part bends and where it is stressed under loads: a linear static
analysis (small displacements, linear elastic, isotropic material).

- **Holding and loading:** `FIX FACE x,y,z` holds the flat face nearest the point still;
  `LOAD FACE x,y,z FORCE fx,fy,fz` pushes on one with a force in newtons, spread evenly over
  it. `NEAR x,y,z r` picks the surface within `r` of a point instead. Give as many as needed.
- **Material:** steel by default; `FEA MATERIALS` lists the others (stainless, aluminium,
  titanium, brass, copper, ABS, PLA, PETG, nylon), or give `E`, `NU` and `YIELD`.
- **Results:** the largest von Mises stress and where, the largest displacement and where,
  and the factor of safety (yield over that stress).
- **PLOT** colours a copy of the body by stress, blue to red, with a legend (the body itself
  is hidden); `DEFORM s` also shows its shape bent `s` times over. An analysis with a plot
  goes into the feature history, so it is worked out again when the model changes.
- **How:** the body is filled with cubes (`CELL`, about 40 across its longest side by
  default, at most 120,000 of them), each an 8-node element; the equations are solved by
  conjugate gradients. A 100 x 10 x 10 mm steel cantilever with 100 N on its end comes out
  at 0.1995 mm and 61.7 MPa (with `CELL 1`), against 0.200 mm and 60 MPa from beam theory.
  Finer cubes resolve small features (fillets, thin walls) better; stresses right at a
  held face or a sharp inside corner run high, as they do in any such analysis.

```text
BOX 100 10 10 @50,0,0
HOLE "Block 1" AT 30,0,10 DIAMETER 4 THROUGH
FEA "Block 1" FIX FACE 0,0,5 LOAD FACE 100,0,5 FORCE 0,0,-100 CELL 1 DEFORM 50
```

**More loads:**

- **PRESSURE** pushes on every cube face of the face picked (`p` times its area). A negative
  pressure pulls.
- **BEARING** spreads a pin's push over the side of a hole facing the push, most where it
  faces it squarely (a cosine spread). Give the hole's centre, its radius and the force.
- **GRAVITY** loads every cube with its weight (the material's density).
- Loads of all kinds add up in one analysis. On the cantilever above:
  - Its own weight sags its tip 0.000576 mm. Beam theory gives `w L^4 / (8 E I)` = 0.000578.
  - 1 MPa on its top face bends it 0.748 mm, against 0.750.

**Natural frequencies:**

- `MODES n` finds the lowest `n` (up to 12), with the body held as fixed. The method is
  block inverse iteration with Rayleigh-Ritz on the stiffness and lumped mass.
- `MODE k` plots mode `k`'s shape: the body bent along it, coloured by how far each point
  moves.
- On the cantilever, the first bending mode comes out at 827.7 Hz with the default 2.5 mm
  cubes, and 815.3 Hz with `CELL 1`. Beam theory gives 815 Hz. There are two such modes,
  one each way across the square section. The second bending mode is 6 times higher.
- A body that isn't held has no frequencies here: FIX it as it is mounted.

**Heat:**

- `TEMP FACE x,y,z C` holds a face at a temperature, and `HEAT FACE x,y,z W` puts heat in
  (negative takes it out). The steady temperatures follow from the material's conductivity.
  They are reported, and with `PLOT`, coloured on the body.
- With **THERMAL** the static analysis adds the stress of the body's expansion, from
  `REF` (20 C by default) to those temperatures.
- A steel bar 100 mm long, warmed by 100 C and held at one end, grows 0.12 mm (alpha dT L),
  free of stress.

```text
FEA "Block 1" FIX FACE 0,0,5 MODES 3
FEA "Block 1" FIX FACE 0,0,5 GRAVITY PRESSURE FACE 50,0,10 0.5
FEA "Block 1" TEMP FACE 0,0,5 20 HEAT FACE 100,0,5 5 MATERIAL aluminium PLOT
```

**Reports:** `REPORT file.html` writes the analysis up as one page. Open it in any browser,
or print it to PDF. The page holds:

- **Summary:** the verdict (the factor of safety, and whether the part holds, has a thin
  margin or yields), the peak stress and displacement and where they are, the first natural
  frequency, the mass and whether the solver converged.
- **Inputs:** the material, the cubes, and every fixing and load.
- **Results:** the frequencies, and pictures of the results on the part with their colour
  scales. The pictures show the stress, the displacement (the shape exaggerated), the
  temperatures, and the first three mode shapes.

```text
FEA "Block 1" FIX FACE 0,0,5 LOAD FACE 100,0,5 FORCE 0,0,-100 MODES 2 REPORT bracket-analysis.html
```

## Measure

```text
MEASURE x,y,z x,y,z
MEASURE "body" "body"
```

- **Two points:** the distance between them and its components.
- **Two bodies:** the closest distance between them, and the two closest points. It checks each
  body's corners against the other's triangles, both ways.

## Variables, equations and configurations

```text
VAR name = expression            a variable, or an equation of others
VAR [LIST] | VAR DELETE name
CONFIG SAVE name                 the variables' values, kept as a configuration
CONFIG name                      back to one
CONFIG TABLE file.csv            a design table: one configuration a row
CONFIG [LIST] | CONFIG DELETE name
```

Any command takes `{expression}` wherever it takes a number: `BOX {w} {d} 10`, `RECT 0,0
{w},{w/2}`, `HOLE AT 0,0,{h} DIAMETER {r*2} THROUGH`. The feature history keeps the
braces, so changing a variable rebuilds the model from its history with the new values
(refused, with everything kept as it was, when the rebuild fails).

- **Expressions:** numbers, variables, `+ - * / ^` and parentheses, `sin cos tan` (degrees),
  `asin acos atan`, `sqrt abs floor ceil round min max pow` and `pi`. Values are in the
  document's units, as typed numbers are.
- Variables may use each other (`VAR d = w / 2`), but not round in a circle. One the history
  still uses can't be deleted.
- **Design table:** a CSV file; its first row is a heading for the configurations' names,
  then the variables' names; each further row is a configuration.
- Variables and configurations are saved with the project.

```text
VAR w = 60
VAR d = w / 2
BOX {w} {d} 10
CONFIG SAVE standard
VAR w = 100                      the block grows to 100 x 50
CONFIG standard                  and back
```

## Feature history

Every model command that succeeds is recorded in order: sketches, solids, features, cuts,
sculpting, moves, selections and plane changes.

```text
HISTORY                         the steps (the feature tree)
HISTORY EDIT n command          change a step, and rebuild
HISTORY SET n KEY value ...     change one or more of a step's parameters, and rebuild
HISTORY INSERT n command        add a step before step n, and rebuild
HISTORY DELETE n                remove a step, and rebuild
HISTORY MOVE n TO m             reorder: step n becomes step m, and rebuild
HISTORY SUPPRESS n [m | a-b]    keep steps but leave them out of the model
HISTORY UNSUPPRESS n [m | a-b]  put them back
HISTORY ROLLBACK n              build only steps 1 to n; new steps go in there
HISTORY ROLLFORWARD             build every step again
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

**The feature tree:**

- The **Features** list at the top of the Model panel shows the steps.
- Untick a step to suppress it. Double-click a step to edit its command.
- Right-click a step to edit, suppress, move up or down, roll back to it, or delete it.
- Suppressed steps and steps below the rollback bar are greyed. **Roll forward** on the bar
  builds them again.

```text
> HISTORY SET 2 DIAMETER 10
Step 2 is now HOLE "Block 1" AT -20,0,15 DIAMETER 10 THROUGH; regenerated 3 steps
> HISTORY ROLLBACK 1
Rolled back to step 1 of 3: new steps go in there; HISTORY ROLLFORWARD builds the rest
> FILLET "Block 1" RADIUS 2 EDGES NEAR 0,20,15        (goes in as step 2)
> HISTORY ROLLFORWARD
```

- A suppressed or moved step can leave a later step without what it needs, such as a hole on
  a body that no longer exists. The change is then refused, and the report names the step that
  failed.
- Suppressed steps and the rollback point are saved with the project.
- While rolled back, UNDO takes back the step just above the bar.

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
- `constrain`, `dim`, `constraints`, `constraint_edit`, `constraint_delete`, `drag`;
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
{"operation": "drag", "curve": "Polyline 1", "grip": "point 3", "to": [60, 50]}
{"operation": "massprops", "body": "Housing", "material": "aluminium"}
{"operation": "history_edit", "step": 2, "command": "HOLE \"Block 1\" AT -20,0,15 DIAMETER 10 THROUGH"}
```

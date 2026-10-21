# Part features and the feature history

These features complete the solid modelling tools. They work on sketch curves and on bodies,
from *Design* in the menu, the command line, or the `cad` MCP tool.

| Kind | Features |
| --- | --- |
| Solids from sketch curves | `EXTRUDE`, `REVOLVE`, `SWEEP` and `LOFT` |
| Copies of bodies | `PATTERN` and `MIRROR` |
| Edge rounds | `FILLET` |
| Analysis | `MASSPROPS` and `MEASURE` |

Every model command goes into a **feature history** that can be edited and regenerated.

![A revolved vase, a loft through three sections, a swept pipe, and a rounded block with a pattern of holes](cad-features.png)

The picture comes from [examples/cad-demo.txt](examples/cad-demo.txt) (`RUN docs/examples/cad-demo.txt`).

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

## Fillet (rounds)

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
- It rounds all the edges of the kind chosen. Rounding chosen edges only would need the
  boundary-representation edges that meshes don't have.

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
- `pattern`, `mirror`, `fillet`;
- `massprops`, `measure`;
- `history`, `regen`, `history_edit`, `history_insert`, `history_delete`.

```json
{"operation": "revolve", "curve": "Polyline 1", "axis": "v"}
{"operation": "pattern", "body": "Boss", "type": "circular", "count": 6, "axis": "z"}
{"operation": "fillet", "body": "Housing", "radius": 2, "edges": "both"}
{"operation": "massprops", "body": "Housing", "material": "aluminium"}
{"operation": "history_edit", "step": 2, "command": "HOLE \"Block 1\" AT -20,0,15 DIAMETER 10 THROUGH"}
```

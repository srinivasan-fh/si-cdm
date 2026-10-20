# Digital sculpting and cuts

Any body can be shaped like clay and cut like a CAD solid. The two tool sections at the bottom of the
tool rail hold the tools, the options bar holds their settings, and every edit can be undone (`UNDO`,
or the **Undo** button in the options bar).

![A clay hump on a plate with drilled holes, a channel, a pocket and a boss](sculpting.png)

## Sculpting brushes

Drag over a body. **Ctrl** inverts the brush, **Shift** smooths with any brush, `[` and `]` change the size.

| Brush | Does | Ctrl |
| --- | --- | --- |
| **Draw** | Pulls the surface out along the brush normal, like pressing clay from behind | Pushes in |
| **Clay** | Builds material in flat layers, up to a plateau above the surface | Removes in layers |
| **Carve** | Digs material away with a sharp falloff | Raises a ridge |
| **Inflate** | Swells the surface along its own normals | Deflates |
| **Grab** | Picks up the surface under the brush and drags it with the mouse | |
| **Pinch** | Draws the surface towards the stroke, for creases and hard-edge transitions | Spreads |
| **Crease** | Cuts a sharp valley along the stroke | Sharp ridge |
| **Smooth** | Laplacian smoothing: each vertex moves towards the average of its neighbours | |
| **Flatten** | Presses the surface onto the average plane under the brush | |
| **Scrape** | Shaves off the high spots down to that plane, like a trimming knife | Fills the low spots |

The settings are radius, strength (0 to 1), **Symmetry X** (mirrors the stroke across the body's centre)
and **Dyntopo**.

**Dynamic topology.** Each dab first splits every edge under the brush that is longer than the detail
size (default: a quarter of the radius). The splits stay conforming, so the surface stays closed. A
12-triangle block can be sculpted as freely as a dense scan.

**How a dab works.**

- Every vertex within the radius *R* of the hit point moves.
- The amount is weighted by a smooth falloff, *f* = 3t² − 2t³ with *t* = 1 − d/R.
- The brush plane is the falloff-weighted average position and normal under the brush.
- Dabs are spaced a quarter radius apart along the stroke, each snapped to the surface.

Hard CAD edges stay crisp in the shading: normals are smoothed only across edges that turn less than 40°.

## Cuts and booleans

| Tool | Use |
| --- | --- |
| **Drill Hole** | Click a surface. Through, or blind to a depth with a 118° drill point. Drills along the surface normal |
| **Pocket** | Click a surface. Box (width, length, depth, angle), round, or ball end |
| **Channel** | Drag along a surface. A round-bottomed groove follows the stroke (cable runs, vents) |
| **Knife / Trim** | Drag a line across a body. The plane through it cuts the body, the smaller side goes, and the cut face is capped |
| **Add Volume** | Click a surface to fuse on a cylinder boss, a square pad or a dome |
| **Boolean Bodies** | Click the body to keep, then the tool body. Union, subtract or intersect. The tool body is used up unless *Keep* |

**How the booleans work.** They are exact, and the result is a closed mesh. Each operation runs these steps:

1. **Snap to a grid.** Both meshes go onto a shared integer grid, 2²⁶ steps across the larger one. That
   is finer than their float coordinates.
2. **Find the intersection segments.** Every pair of triangles that cross gives a segment.
   - The ends are canonical points: an edge of one mesh crossing the plane of a triangle of the other.
   - Each point is computed once and shared by every triangle that meets it, so the two sides of a seam
     match exactly.
   - Which side of a plane a vertex is on, and whether a crossing is inside a triangle, are decided with
     exact 128-bit determinants. Neighbouring triangles never disagree.
3. **Re-triangulate.** Each cut triangle is triangulated again, with its segments as constraints. The
   constraints are inserted by edge flips with exact orientation tests.
4. **Keep or drop each region.** The regions between seams are classified by their generalised winding
   number in the other mesh. That number is exact near the point and approximated far from it.

**Results.** Holes, pockets, bosses and two-body booleans on clean solids come out with the exact
volume (for example a Ø20 through hole removes exactly its 48-sided prism) and with no open edges.

**Limits.** Self-intersecting input can leave a few open edges. A brush stroke folded through itself
is an example. The command reports them ("N open edges"), and `UNDO` restores the body.

**Export.** Results are ordinary triangle meshes. `EXPORT part.stl` sends them to a slicer for 3D
printing, or to CAM for CNC machining.

## Commands

```text
SCULPT [body] brush [AT] x,y,z [x,y,z...] [RADIUS r] [STRENGTH 0-1] [INVERT] [SYMMETRY] [DETAIL d]
       [NOREFINE] [MOVE dx,dy,dz]          brushes: DRAW CLAY CARVE INFLATE GRAB PINCH CREASE SMOOTH FLATTEN SCRAPE
SCULPT [body] REFINE edge-length           refine the whole body before sculpting
SCULPT LIST
HOLE    [body] [AT] x,y,z [DIR x,y,z] DIAMETER d [DEPTH h | THROUGH]
POCKET  [body] [AT] x,y,z [DIR x,y,z] [BOX|ROUND|BALL] WIDTH w [LENGTH l] DEPTH d [ANGLE deg]
CHANNEL [body] [PATH] x,y,z x,y,z ... WIDTH w [DEPTH d]
TRIM    [body] [PLANE] x,y,z NORMAL x,y,z [SMALLER]        removes the side the normal points to
BOSS    [body] [AT] x,y,z [DIR x,y,z] [CYLINDER|BOX|BALL] DIAMETER d HEIGHT h
BOOLEAN body UNION|SUBTRACT|INTERSECT other [KEEP]
UNDO
```

Points are snapped to the nearest surface, and the tool goes in against the surface normal there
(unless `DIR`). The body defaults to the selection. A point far from the body is refused. Each command
reports the triangle count and whether the result is closed.

Over MCP: the `sculpt` tool (`operation` = a brush name, `hole`, `pocket`, `channel`, `trim`, `boss`,
`boolean`, `refine` or `undo`).

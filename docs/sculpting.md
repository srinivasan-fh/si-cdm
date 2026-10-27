# Digital sculpting and cuts

Any body can be shaped like clay and cut like a CAD solid. The two tool sections at the bottom of the
tool rail hold the tools, the options bar holds their settings, and every edit can be undone and
redone (`UNDO`, `REDO`, or the buttons in the options bar).

The toolset follows the common ground of the professional sculpting tools: ZBrush, Blender,
3DCoat, Mudbox and Nomad Sculpt. It covers:

- eighteen brushes;
- falloff curves;
- mirror and radial symmetry;
- masking;
- alphas;
- polypaint and polygroups;
- dynamic topology;
- voxel remesh;
- subdivision, symmetrize and decimation;
- undo and redo.

The code is our own. No code from those tools is used.

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
| **Clay Strips** | Lays flat-topped strips of clay under a square brush, along the stroke | Removes |
| **Layer** | Raises the surface to an even height over the stroke, at most once a stroke however often it passes | Lowers |
| **Blob** | Swells rounded blobs that spread out from the centre | Dents |
| **Snake Hook** | Pulls the surface out along the stroke, for horns, tentacles and strands; it keeps the strand's girth | |
| **Nudge** | Pushes the surface along itself in the stroke's direction | |
| **Fill** | Raises the low spots up to the plane under the brush | Scrapes |
| **Mask** | Paints a mask; masked surface (shown darker) holds still under every other brush | Unmasks |
| **Paint** | Polypaint: lays colour on the surface; the mask holds it off | Paints the body's own colour back |

![A mask (the dark band) holds the surface still under a Draw stroke across it; a Snake Hook pulls out a strand](sculpt-tools.png)

The settings in the options bar:

- radius and strength (0 to 1);
- **Symmetry X / Y / Z**: mirrors each dab across the body's centre planes, in any combination;
- **radial count**: copies of each dab around the body's vertical axis, for wheels, flowers and
  rosettes;
- **falloff**: smooth, sphere, sharp, linear or flat, from the centre to the rim;
- **Dyntopo**.

A copy that lands almost on another, such as a dab on a mirror plane, is left out, so the plane
does not get double strength.

**Front faces only.** A dab moves only the surface that faces the way its own spot does. A brush
larger than a thin part, such as an ear or a 10 mm slab, does not reach through to the far side or
pull its plane there.

**Masking.** The Mask brush paints a mask from 0 to 1 on the vertices. `MASK CLEAR`, `INVERT`,
`SMOOTH`, `GROW`, `SHRINK` and `ALL` edit the whole mask. A masked vertex moves by (1 − mask) of
what the brush would do. The mask follows the surface through dynamic topology and is saved with
the project.

**Subdivide, symmetrize, decimate.**

- `SUBDIVIDE` is Loop subdivision. Each level splits every triangle in four and smooths: corners
  round off, and borders stay put.
- `SYMMETRIZE` cuts the body on the plane through its origin, mirrors the kept side exactly and fuses
  the halves into one closed solid.
- `DECIMATE TO n` collapses the shortest edges until about n triangles are left. The surface stays
  closed and its volume stays within a few per cent.

**Dynamic topology.** Each dab first splits the edges that pass under the brush and are longer than
the detail size (default: a quarter of the radius), and the longest edge of each triangle it touches.
After the dab it collapses edges under the brush that became much shorter than the detail. The splits
stay conforming, so the surface stays closed. A 12-triangle block can be sculpted as freely as a dense
scan.

- One dab may add only about as many triangles as its own disc needs.
- A stroke has at most about 1000 dabs.
- The brush is never smaller than 0.2 % of the body's size, so a tiny radius cannot run away.
- Slivers that a boolean leaves (a corner lying on the far edge, or two corners a hair apart) are
  flipped or merged first, so a refined boolean result stays closed.

**How a dab works.**

- Every vertex within the radius *R* of the hit point moves.
- The amount is weighted by a smooth falloff, *f* = 3t² − 2t³ with *t* = 1 − d/R.
- The brush plane is the falloff-weighted average position and normal under the brush.
- Dabs are spaced a quarter radius apart along the stroke, each snapped to the surface.

Hard CAD edges stay crisp in the shading: normals are smoothed only across edges that turn less than 40°.

## Alphas

![Stamps through the built-in alphas: dots, scales, bricks, a ring, a weave and crosshatch](sculpt-alphas.png)

An alpha is a grey pattern the brush is pressed through: white gets the brush's full effect,
black none. Any brush takes one (Draw, Clay, Inflate, Carve, Paint...), for scales, bricks,
pores, rivets or a logo.

- **Built in:** `NOISE`, `RING`, `SQUARE`, `DOTS`, `SCALES`, `CROSSHATCH`, `BRICKS`, `PORES`,
  `WEAVE` and `STRIPES` (`SCULPT ALPHAS` lists them).
- **Your own:** `ALPHA "pattern.png"` reads any image (PNG, JPEG, BMP, TGA...) as an alpha by
  its brightness. Transparent parts count as black.
- **Placing it:** the alpha covers the brush's square, laid on the surface under it.
  - `ALPHAANGLE deg` turns it.
  - `FOLLOW` turns it along the stroke.
  - `TILE n` repeats it n times across.
- **Stamps:** a single point is one stamp. `SPACING 2` sets stamps side by side along a
  stroke (the gap between dabs, in radii; strokes default to a quarter).
- **The falloff still applies,** spread over the square: `FALLOFF CONSTANT` gives a crisp,
  even stamp, and `SMOOTH` fades it out to the edges.
- **Detail:** under an alpha, dynamic topology divides the surface to about a thirtieth of
  the radius, so the pattern comes out. Larger brushes give larger, cleaner patterns.
- **In the app:** the options bar has an alpha list, its angle and *Follow*. *Stamp* drags
  out one stamp: press where it goes, then drag to size and turn it.

```text
SCULPT "Armour" DRAW AT 0,0,40 RADIUS 20 STRENGTH 0.35 ALPHA scales FALLOFF constant
SCULPT "Wall" DRAW AT -40,0,10 40,0,10 RADIUS 12 STRENGTH 0.4 ALPHA bricks SPACING 2 FALLOFF constant
SCULPT "Head" DRAW AT 0,-30,60 RADIUS 8 STRENGTH 0.1 ALPHA pores SPACING 0.6
SCULPT "Badge" DRAW AT 0,0,5 RADIUS 15 STRENGTH 0.3 ALPHA "logo.png"
```

## Polypaint and polygroups

![A painted sphere, and a box shown in its polygroups](sculpt-paint.png)

**Polypaint** colours the surface vertex by vertex. A painted model carries its colour into
renders, OBJ files and 3D printers that print in colour.

- **The Paint brush** (the brush beside Mask, with a colour box in the options bar) lays its
  colour on, blending by strength and falloff. **Ctrl** paints the body's own colour back.
- **Fill:** `PAINT [body] FILL colour` paints the whole body, or one polygroup with
  `GROUP n` or `GROUP AT x,y,z`. `PAINT CLEAR` takes the paint off.
- **The mask holds paint off,** as far as it is masked, under the brush and under a fill.
- **Colours** are names (red, green, blue, yellow, orange, purple, pink, cyan, white, black,
  grey, brown, skin, bone, gold, silver, steel, copper, clay), `#RRGGBB`, or `r,g,b` (0 to 1,
  or 0 to 255).
- **Resolution:** paint is as fine as the surface is divided. Refine or subdivide a coarse
  body before painting detail; dynamic topology carries the paint with the surface as you
  sculpt on.

**Polygroups** give each face a group number: a way to pick out parts of a surface.

- `POLYGROUP AUTO [ANGLE 20]` groups faces that meet at less than the angle: a box gets six
  groups, a cylinder three.
- `POLYGROUP ISLANDS` groups each connected piece, `POLYGROUP MASK` makes the masked faces a
  group of their own, and `POLYGROUP PAINT` groups regions of one colour.
- `MASK GROUP n` (or `MASK GROUP AT x,y,z`) masks exactly one group, ready to sculpt the rest
  or that group alone (`INVERT`).
- **View > Polygroups** (or `POLYGROUP SHOW` / `HIDE`) draws each group in its own colour.
- Groups follow the faces through dynamic topology, subdivision and decimation.

**Kept and exchanged:**

- Paint, groups and masks are saved in the project.
- OBJ export writes paint as vertex colours (`v x y z r g b`) and groups as OBJ groups
  (`g`). ZBrush, Blender and MeshLab read both, and OBJ import reads them back.
- A painted or grouped CAD body becomes a mesh body of the same shape. It keeps its exact
  geometry for STEP.

## Voxel remesh

`REMESH` (like ZBrush's DynaMesh or Blender's voxel remesh) rebuilds a body as one closed solid with
evenly sized triangles. Given several bodies, it fuses them, so a form can be blocked out from
overlapping primitives and then sculpted.

- `VOXEL v` sets the voxel size, and the new triangles are about that size. The default is the parts'
  size / 160.
- `BLEND b` fills in where parts meet, with a fillet of about `b` mm. Elsewhere each surface stays
  where it was.
- `SMOOTH s` rounds over detail finer than `s` mm, such as the flats of a faceted primitive (default:
  one voxel).
- Open or self-overlapping input works. Inside is whatever a flood from outside cannot reach.
- The other bodies are used up. The result keeps the first body's name, colour and place.

How it works:

1. Each part's exact distance is computed in a thin band of voxels at its surface.
2. Each voxel takes the nearest surface point its neighbours know. Two raster sweeps carry this out
   to the fillet's reach in linear time.
3. The parts' signed distances are joined with a smooth minimum, then lightly blurred.
4. The surface at distance 0 is extracted with marching tetrahedra, smoothed, and coarsened to the
   voxel size.

`SCALE body sx,sy,sz` stretches a body along x, y and z, and `ROTATE body rx,ry,rz` tilts it. Both
work about the body's origin, which is the base point of a primitive. Together with `SPHERE` they
give ellipsoids of any proportion to block out with.

**Strokes on dense meshes.** A stroke works on the part of the surface it can reach: the triangles
within twice the brush radius of its path, and of its mirror. The edges on the window's border are
pinned, so the surface stays closed. On a 150,000-triangle head, the dabs cost about as much as on a
small mesh; only copying the surface in and out goes over all of it.

## Example: a head bust

![A female head-and-shoulders bust sculpted in SI-CDM: skull, eyes under their lids, nose, lips, cheeks, ears, neck muscles, collarbones and shoulders](head-bust.png)

[examples/head-bust.txt](examples/head-bust.txt) sculpts this bust in 92 commands. Run it in the app
(it opens a new file tab and takes about 25 s):

```text
RUN docs/examples/head-bust.txt
```

It works the way a sculptor works in clay:

1. **Block out.** Twelve stretched balls give the big forms:
   - the skull, the face and the mouth over the teeth;
   - a broad forehead, the lower jaw and its sides, the chin;
   - the cheekbones and the cheeks.

   `REMESH ... BLEND 24` fuses them into one smooth solid. The forms flow into each other with no
   seams, and the mesh is even.
2. **Features.** A grab pushes in the eye sockets. Then the eyeballs, the nose and the ears are added
   as smaller forms. The nose is a bridge, a body, a tip and two wings, fused with each other first
   (`BLEND 7`) so it reads as one form. They are fused on with tight 3 mm fillets.
3. **Neck and shoulders.**
   - The neck is a cylinder leaning forward, deeper than it is wide.
   - The shoulders, their caps and the chest are stretched balls.
   - They are fused with 30 mm fillets, cut flat underneath, and joined to the head with an exact
     union. The join under the jaw is smoothed.
4. **Detail with the brushes:**
   - brows;
   - eyes: the upper lid drawn over the ball, its fold, the lower lid and the inner corner;
   - mouth: the lips with the upper lip's bow, the line between them, the corners and the groove
     under the lower lip;
   - ears: the bowl, the rim and the inner ridge;
   - neck: the muscles from behind the ears to the notch, the notch, the collarbones.

   Strokes use `SYMMETRY`, so each is placed once.

Points in a script only need to be near the surface. Each one is snapped onto it, and a stroke runs
along the surface between the snapped points. The result is one closed body of about 160,000
triangles, ready for `EXPORT bust.stl`.

The test suite runs the script and checks the result:

- it is closed;
- the nose stands out over the lips;
- the mouth line lies between the lips;
- the eyes sit under the brow;
- the ears stand out from the skull;
- the two sides of the face match to within 1 mm.

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

**Faces in contact.** Bodies that touch face to face (blocks side by side, a slab cut off the end of a
block) are handled with the coplanar rule:

- Where both solids lie on the same side of a shared face, one copy stays in a union or intersection.
- Where they meet back to back, both faces go.

A region's test point is never taken on such a face. Faces that only partly overlap go through a
retry with the tool nudged by a hundred-thousandth of its size. That result is closed, and its
volume is accurate to about 1e-5.

**Precision.** Booleans and cuts run in the body's own frame, so a part far from the origin is cut
as exactly as one at it.

**Limits.**

- Self-intersecting input can leave a few open edges, for example a brush stroke folded through
  itself. The command reports them ("N open edges").
- A cut that would add material, or an addition that would remove it, is refused, and the body is
  left unchanged.
- A channel path that comes back within its own width is refused. Draw it as separate channels.

**Undo.**

- `UNDO` steps back through the last 16 shape edits (sculpt, cuts, booleans, armour, joints, shells,
  lattices), within about 2 million vertices of history.
- A joint's swing goes with its pin.
- Moves, colours and renames made since the edit are kept.
- If bodies were added, removed or rebuilt since the edit, `UNDO` says so and does nothing, rather
  than sweep them away.

**Export.** Results are ordinary triangle meshes. `EXPORT part.stl` sends them to a slicer for 3D
printing, or to CAM for CNC machining.

## Commands

```text
SCULPT [body] brush [AT] x,y,z [x,y,z...] [RADIUS r] [STRENGTH 0-1] [INVERT] [SYMMETRY [X|Y|Z...]]
       [RADIAL n [X|Y|Z]] [FALLOFF smooth|sphere|sharp|linear|constant] [DETAIL d] [NOREFINE] [MOVE dx,dy,dz]
       [COLOR colour] [ALPHA name|"image" [ALPHAANGLE deg] [FOLLOW] [TILE n]] [SPACING radii]
       brushes: DRAW CLAY CLAYSTRIPS LAYER CARVE INFLATE BLOB GRAB SNAKEHOOK NUDGE PINCH CREASE
                SMOOTH FLATTEN SCRAPE FILL MASK PAINT
SCULPT [body] REFINE edge-length           refine the whole body before sculpting
SCULPT LIST                                the brushes
SCULPT ALPHAS                              the built-in alphas
HOLE    [body] [AT] x,y,z [DIR x,y,z] DIAMETER d [DEPTH h | THROUGH]
POCKET  [body] [AT] x,y,z [DIR x,y,z] [BOX|ROUND|BALL] WIDTH w [LENGTH l] DEPTH d [ANGLE deg]
CHANNEL [body] [PATH] x,y,z x,y,z ... WIDTH w [DEPTH d]
TRIM    [body] [PLANE] x,y,z NORMAL x,y,z [SMALLER]        removes the side the normal points to
BOSS    [body] [AT] x,y,z [DIR x,y,z] [CYLINDER|BOX|BALL] DIAMETER d HEIGHT h
BOOLEAN body UNION|SUBTRACT|INTERSECT other [KEEP]
UNDO
REMESH  [body ["other"...]] [VOXEL v] [BLEND b] [SMOOTH s]   voxel remesh: fuse into one even, closed solid
SCALE   body sx,sy,sz                      stretch along x, y, z (about the body's origin)
ROTATE  body rx,ry,rz                      orient: degrees about X, then Y, then Z
MASK    [body] CLEAR|INVERT|SMOOTH|GROW|SHRINK|ALL [n]   edit the mask (the MASK brush paints it)
MASK    [body] GROUP n | GROUP AT x,y,z [INVERT]       mask one polygroup
PAINT   [body] FILL colour [GROUP n | GROUP AT x,y,z] | CLEAR    polypaint (the PAINT brush paints strokes)
POLYGROUP [body] AUTO [ANGLE deg] | ISLANDS | MASK | PAINT | CLEAR | LIST | SHOW | HIDE
SUBDIVIDE [body] [LEVELS n]                Loop subdivision
SYMMETRIZE [body] [X|Y|Z] [-]              mirror the + side (or the - side) across the body's origin
DECIMATE [body] TO n | RATIO r             reduce to about n triangles
REDO                                       put back what the last UNDO took away
RUN script.txt                             the commands in a file, one a line (# comments); stops at a failure
```

Points are snapped to the nearest surface, and the tool goes in against the surface normal there
(unless `DIR`). A stroke runs along the surface between its snapped points. The body defaults to
the selection. A point far from the body is refused. Each command reports the triangle count and
whether the result is closed.

Over MCP: the `sculpt` tool (`operation` = a brush name, `hole`, `pocket`, `channel`, `trim`, `boss`,
`boolean`, `refine`, `remesh`, `subdivide`, `symmetrize`, `decimate`, `mask_clear`, `mask_invert`,
`mask_smooth`, `mask_group`, `paint_fill`, `paint_clear`, `polygroup_auto`, `polygroup_islands`,
`polygroup_mask`, `polygroup_paint`, `polygroup_clear`, `undo` or `redo`; `symmetry_axes`, `radial`,
`falloff`, `color`, `alpha`, `alpha_angle`, `alpha_follow`, `alpha_tile` and `spacing` for the brushes).

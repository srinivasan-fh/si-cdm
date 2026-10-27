# Armour and exoskeletons

Wearable parts start from a human or robot base. You sculpt or model a shape, turn it into a hollow
part that fits, joint it, lighten it and check that it clears the body as it moves. The tools are in
the **Armour & Exoskeleton** section of the tool rail. Each one is also a command and part of the MCP
`armor` tool. Every edit can be undone (`UNDO`).

![Armour over the humanoid robot: a merged chest and shoulder plate, forearm, upper-arm, thigh and shin guards, and a shelled pelvis](armour.png)

## Tools

| Tool | Use |
| --- | --- |
| **Shell / Hollow** | Click a body. A solid is hollowed to the wall thickness, or a surface is solidified into a plate. Choose inward or outward. With *Open*, the side you click is cut away first, which gives a helmet, a gauntlet or a tray |
| **Armour Offset** | Click a body part, such as a humanoid's forearm or chest. You get a closed shell around it. Its inner face is the clearance away from the part, which leaves room for padding, wiring and movement. The wall is as thick as you set. The shell is attached to the part and moves with it |
| **Joint / Hinge** | Click a part at its pivot. A hinge pin or ball joint goes there, with a motion that swings the part through its range against the part it touches |
| **Lighten (Lattice)** | Click a thick part. A solid skin is kept and the inside is filled with a gyroid lattice, or left hollow |
| **TPMS Lattice** | The same gyroid infill, from the *Generative* section |
| **Interference Check** | Sweeps the motion timeline (gestures, walk, joints) and lists every pair of parts that collide or come closer than the clearance |

## How they work

**Shell.** The surface is offset along angle-weighted vertex normals. Where faces meet at an angle,
the offset is lengthened so the wall keeps its thickness at edges. The inner and outer faces are then
bridged along any open border. *Open* first trims the body with the exact boolean knife and removes
the cap, so the opening is a clean cut. A 100 × 80 × 60 box hollowed to a 4 mm wall comes out at
exactly 100·80·60 − 92·72·52 mm³ and closed.

**Armour.**

- The part goes into a voxel grid about 160 cells across, fine enough for the wall.
- Each voxel near the part gets the exact distance to its surface.
- Inside and outside come from a flood fill from the grid's border. The fill stops at the layer of
  voxels that touch the surface. This is a topological test, so open or overlapping robot shells
  (most real parts) still get a clean outside.
- The signed distance is blurred slightly. That rounds over ribs, vents and bolts finer than the wall,
  which an exact offset would turn into cusps.
- The inner face (distance = clearance) and the outer face (distance = clearance + thickness) are
  extracted separately with marching tetrahedra, smoothed (Taubin), simplified and joined. Two level
  sets of one field never cross, so the shell is closed even when the wall is only a voxel or two
  thick.

**Limits.**

- The wall must be at least half a voxel thick. That is about 2 mm on the humanoid's chest plate
  and about 1 mm on a forearm; the command names the minimum.
- A large clearance costs little: exact distances are computed only near the surface and the two
  faces, and a coarse pass bounds the rest.

`MERGE` makes one shell around several parts, for example a chest plate over the chest and both
shoulder motors. The armour is named `Armor <part>`, remembers the part it is attached to (saved with
the project), and takes the part's transform in every motion and in physics.

**Joints.**

- A hinge adds a pin, a cylinder along the axis. A ball joint adds a sphere.
- Either one is attached to the parent part.
- The child gets a looping motion: 0 → upper limit → lower limit → 0 over the period, with hard joint
  limits.
- If the parent moves, it carries the child with it.

**Lighten.**

- The solid is voxelised. A column-parity test decides inside and outside.
- The depth below the surface is measured by a chamfer distance transform.
- Below the skin, a gyroid field `sin x cos y + sin y cos z + sin z cos x` is thickened to the
  lattice wall.
- The void is marched, smoothed and simplified, then added to the body as an inner, inverted surface.

`LIGHTEN` checks what its voxels can resolve. The skin must be at least half a voxel. The cell must
be at least four voxels. The wall must be at least half a voxel and less than half the cell. Spheres,
cylinders, cones and tori count as closed: their seams are welded with a tolerance.
`SHELL` refuses an inward wall that would leave no cavity, and suggests `OUTWARD` instead.

An 80 × 80 × 40 block with a 2 mm skin, a 12 mm cell and a 1.2 mm wall comes out 60–80 % lighter. It
stays closed and printable. Its outer surface is untouched.

**Interference.**

- The check samples the timeline every `STEP` seconds (the whole length in 40 steps by default).
- At each time it poses the parts. A box test skips pairs that are far apart.
- Up to 1500 vertices of each part are measured against the other part's surface.
- Near the surface, the distance is signed by which side of the nearest face the vertex is on. At
  concave corners and on open robot shells, the winding number decides instead.
- Deeper points get their true depth. Inside or outside is cached in octree-like cells, sized so that
  every point in a cell is on the same side.
- By default every armour part is checked against everything else.
- Pairs that already overlap in the rest pose are counted, not listed, unless the motion pushes them
  deeper. Robot links often overlap at their joints by design.

The report lists the worst pairs first: how deep (or how close), when, and where.

## Commands

```text
SHELL   [body] THICKNESS t [OUTWARD] [OPEN x,y,z NORMAL x,y,z | FACE x,y,z]
ARMOR   [body | robot PART word[, word...] [MERGE]] [CLEARANCE d] [THICKNESS t]
JOINT   child [TO parent] AT x,y,z [AXIS X|Y|Z|x,y,z] [HINGE|BALL] [RANGE lo hi] [PERIOD s] [SIZE d]
LIGHTEN [body] [SKIN t] [CELL c] [WALL w] [HOLLOW]
INTERFERENCE [CLEARANCE d] [STEP s] [DURATION s] [BODIES name...]
UNDO
```

Examples on the humanoid robot (`PART humanoid`). The helmet, visor and back plate stand for bodies
you have modelled:

```text
ARMOR "Male Humanoid Robot 1" PART chest, heart, motor right shoulder, motor left shoulder, motor neck MERGE CLEARANCE 4 THICKNESS 4
ARMOR "Male Humanoid Robot 1" PART forearm CLEARANCE 3 THICKNESS 3       (both forearms)
SHELL "Male Humanoid Robot 1 Pelvis" THICKNESS 3 OUTWARD
SHELL "Helmet" THICKNESS 3 OPEN 0,0,1600 NORMAL 0,0,-1                    (open at the bottom)
JOINT "Visor" TO "Helmet" AT 0,-120,1700 AXIS X RANGE 0 80 PERIOD 3
LIGHTEN "Back Plate" SKIN 2 CELL 12 WALL 1.2
INTERFERENCE CLEARANCE 2
```

Typical output:

```text
Interference over 10.00 s (41 samples, 1683 pairs), clearance 1 mm:
  COLLIDES Male Humanoid Robot 1 Right Shin / Armor Male Humanoid Robot 1 Left Shin: 44.94 mm deep at 1.75 s near (33.11, 8.217, 598.608) mm
  ...
```

The humanoid's default timeline sweeps every joint through its full range at once, so it reports
many collisions. Run a gesture or a walk to check the motions you actually use.

`PART` matches body names that contain any of the words. Motors are skipped unless a word says
"motor", so `PART forearm` armours both forearms and leaves the motors alone. Without `MERGE`, each
match gets its own shell.

Over MCP: the `armor` tool takes an `operation`: `shell`, `armor` (with `part` and `merge`), `joint`,
`lighten` or `interference` (with `bodies`, `step` and `duration`).

**Speed.** On the humanoid, the merged chest plate takes about 5 s. An interference sweep of nine
armour parts against the 183-part robot takes about 14 s.

**Export.** Every result is a closed triangle mesh, ready for `EXPORT part.stl` and a slicer.

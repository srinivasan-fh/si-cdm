# Assemblies

SI-CDM puts parts together the way a fitter does:

- **Mates** set one face against another. A pin goes into its hole, a cover sits on its
  housing, and the solver moves the parts into place.
- **The assembly tree** groups the parts into sub-assemblies. STEP files keep the tree,
  both when they are read and when they are written.
- **The interference check** finds where parts overlap and by how much.
- **The parts list** counts each part.

```text
MATE CONCENTRIC "Plate" AT 19,0,5 "Pin" AT 115,50,24     the pin's axis on the hole's
MATE FLUSH "Plate" AT 0,0,0 "Pin" AT 15,0,-3            its end level with the plate's underside
ASSEMBLY GROUP "Base" "Plate" "Pin"
ASSEMBLY INTERFERENCE
EXPORT base.step
```

In the app, use the **Assembly** menu:

- Choose a mate type.
- Click a face on the part that stays.
- Click a face on the part to move.

The Model panel shows the sub-assemblies as folders, and a **Mates** list. Right-click a mate
to suppress, flip or delete it. A mate that isn't met shows in red.

## Mates

```text
MATE type [value] "A" AT x,y,z "B" AT x,y,z [FLIP]
```

**How faces are picked:**

- A face is picked by a point on it, in mm in world coordinates (what a click in the 3D window
  gives).
- `B` moves to meet `A`.

**What a face gives the mate:**

| Face | Gives |
| --- | --- |
| Flat | A **plane**: a point on it and its outward normal |
| Round: cylinder, cone, torus, revolved | Its **axis** |
| Sphere | Its **centre** point |

**Exact solids** (primitives, extrusions, revolves, STEP and IGES imports) give their faces'
true geometry.

**Meshes** (after a hole, a boolean or a sculpt) use the smooth patch under the point:

- If its normals all point the same way, it is a plane.
- If they all lie square to one direction and the points lie on a circle about it, it is a
  round face. For example, a drilled hole gives its axis.

| Type | Takes | Meets when |
| --- | --- | --- |
| `COINCIDENT` | planes, axes, points | Planes face each other and touch. Axes line up. A point lies on the plane or axis, or on the other point |
| `FLUSH` | planes | The planes are level and face the same way |
| `CONCENTRIC` | axes (or an axis and a sphere) | The axes line up (either way round) |
| `DISTANCE mm` | planes, axes, points | Facing planes this far apart (`FLUSH`-style with `FLIP`); axes or points this far apart |
| `ANGLE deg` | planes, axes | The normals or axes are at this angle |
| `PARALLEL` | planes, axes | The normals or axes are parallel |
| `PERPENDICULAR` | planes, axes | The normals or axes are square to each other |
| `FIX` | a part | The part holds still |

| Command | What it does |
| --- | --- |
| `MATE` | Lists the mates, each met or how far off |
| `MATE FIX "part"` | Holds a part still |
| `MATE EDIT n value` | A new distance or angle |
| `MATE FLIP n` | Planes facing each other become flush, or the other way round |
| `MATE SUPPRESS n` · `UNSUPPRESS n` | Leaves a mate out of the solve, or puts it back |
| `MATE DELETE n` · `MATE CLEAR` | Removes mates. The parts stay where they are |
| `MATE SOLVE` | Places the parts again (after you moved one by hand, say) |

Mates are named M1, M2, and so on. Commands take a mate's name or its number.

### Solving

The solver moves the free parts as little as it can to meet every mate.

- **Parts that hold still:**
  - Parts with `MATE FIX` stay where they are.
  - When none is fixed, the first part mated stays.
  - A part mated earlier moves less readily than one mated later. Mating a new part to an
    assembly moves the new part, not the assembly.
- **Parts that move together:**
  - The bodies of a library component move as one.
  - Bodies attached to a part (its armour, its pins) move with it.

After each solve, the command says:

- which parts moved;
- how many **degrees of freedom** the mates leave. A pin in a hole has 2: it can slide and
  spin. Seated on a face, it has 1. "Fully placed" means 0.
- which mates **can't be met** together.

A new mate that conflicts with the mates already there is refused, and nothing moves.

Mates are saved in the project file. The feature history records `MATE` commands, so REGEN
rebuilds the parts and puts the mates back.

## The assembly tree

```text
ASSEMBLY                                  the tree: sub-assemblies in [ ], parts under them
ASSEMBLY GROUP "Gearbox/Input" "Shaft" "Gear 1"
ASSEMBLY UNGROUP "Gear 1"                 back to the top level
```

**Paths:** a part's sub-assembly is a path. `a/b` is a sub-assembly `b` inside `a`.

**STEP files:**

- **Writing:** STEP export writes each sub-assembly as an assembly product, with the parts placed
  in it (`NEXT_ASSEMBLY_USAGE_OCCURRENCE`). Other CAD systems open it with the same tree.
- **Reading:** STEP import reads the tree of an assembly file. Each part lands in the
  sub-assembly path of its instance.

## Interference

```text
ASSEMBLY INTERFERENCE [part...]
```

**Which parts:** the named parts, or every visible one.

**What it reports for each pair that overlaps:**

- the volume they share (mm³);
- where that volume is;
- the size of its box.

The largest overlaps come first.

**What counts:** parts that only touch, such as a block standing on another or faces held
together by a mate, are not reported. The check counts overlaps of 0.001 mm³ or more, and of
at least a ten-thousandth of the smaller part.

## Parts list

`ASSEMBLY BOM` counts the parts by name. Copies count as the same part: "Bolt", "Bolt 2" and
"Bolt (3)" are one part, quantity 3.

## For AI agents

The MCP tool `assembly` does all of this. Its operations are:

- `mate` (with `type`, `part`, `at`, `moving`, `moving_at`, `value` and `flip`);
- `fix`, `list` and `solve`;
- `edit`, `flip`, `suppress`, `unsuppress`, `delete` and `clear`;
- `tree`, `group` and `ungroup`;
- `interference` and `bom`.

# CNC milling (CAM)

SI CDM makes milling and drilling toolpaths from a part, checks them in a stock simulation, and
posts G-code for the machine's controller. Use *Manufacture* in the menu, the `CAM` command, or
the `cam` MCP tool.

![Toolpaths for a plate with a dome: facing, roughing, parallel and waterline finishing, drilling, profile](cam-toolpaths.png)

A full job (in [examples/cam-demo.txt](examples/cam-demo.txt)):

```text
CAM SETUP "Block 1" STOCK 4 TOP 1 MATERIAL aluminium
CAM TOOL 1 FLAT 8 LENGTH 40
CAM TOOL 2 BALL 6 LENGTH 40
CAM TOOL 3 DRILL 6
CAM TOOL 4 FACE 50
CAM FACE
CAM ROUGH TOOL 1 LEAVE 0.3 BOTTOM 0
CAM PARALLEL TOOL 2 SHALLOW SCALLOP 0.005
CAM WATERLINE TOOL 2 STEEP STEPDOWN 0.3
CAM DRILL CYCLE PECK
CAM CONTOUR TOOL 1 TABS 4
CAM SIMULATE SHOW
CAM POST bracket.nc FANUC
```

`CAM SIMULATE` on this job reports:

```text
Simulated 6 operations on a 0.375 mm grid:
  removed 100.9 cm3; stock left on the part: up to 1.444 mm, 0.032 mm on average
  no gouges
  no rapid moves through stock
  cycle time 28 min 51 s, 54.45 m cutting, 13.09 m rapid
```

The 1.4 mm is the concave corner where the dome meets the plate. A 3 mm ball can't reach into
it.

![The cut stock from the simulation](cam-stock.png)

## How the paths are made

Every milling path starts from the *cutter location*: the lowest height the tool's tip can sit
at, at a point (x, y), without cutting into the part.

- **Drop-cutter:** the cutter location is found exactly for each triangle the tool could touch.
  The facet, edge and corner cases are each solved for flat, ball and bull-nose end mills.
- **Contours:** paths on a level follow where the cutter location crosses that level, using
  marching squares on a grid.
  - Each crossing is refined by bisection on the exact drop-cutter.
  - Corners the grid cut off are put back where they are sharp.
  - Chords that would cut into the part get more points.

So walls come out at exactly the tool radius plus the stock to leave, and rounded corners come
out as exact arcs (`G02 X3. Y46. I3. J0.`).

**Stock to leave** is handled by a cutter grown by that amount, which gives an exact offset
surface.

## Setup

```text
CAM SETUP ["body"] [STOCK margin | STOCK x,y,z | FROM x,y,z TO x,y,z] [TOP mm]
          [MATERIAL m] [ORIGIN CORNER|CENTRE|MODEL] [CLEARANCE mm] [RETRACT mm]
          [TOLERANCE mm] [MAXRPM n] [MAXFEED mm/min] [RAPID mm/min] [POST name]
```

- **Part:** the named body, the selection, or the part already set up.
- **Stock:** defaults to the part's box plus 2 mm around and 1 mm on top. Give one of:
  - `STOCK margin` for a margin all round;
  - `STOCK x,y,z` for a block of that size, centred on the part, from its bottom;
  - `FROM` / `TO` for two corners.

  Running `SETUP` again with only options keeps the stock.
- **Work zero (G54):**
  - `CORNER` (the default): the stock's top, front left corner;
  - `CENTRE`: the stock's top centre;
  - `MODEL`: the model origin.
- **Heights:**
  - The clearance plane (default 15 mm above the stock) is used between operations and at
    tool changes.
  - The retract plane (default 3 mm above the stock) is the highest any link inside an
    operation goes.
- **Materials:** `aluminium`, `steel`, `stainless`, `brass`, `castiron`, `titanium`,
  `plastic`, `wood`. Each has a cutting speed for carbide and for HSS, a chip load, a drill
  feed and a roughing depth.

## Tools and feeds

```text
CAM TOOL n FLAT|BALL|BULL|FACE|DRILL|SPOT|CHAMFER|TAP diameter
         [CORNER r] [FLUTES n] [LENGTH flute-length] [ANGLE deg] [PITCH mm] [HSS] [NAME "x"]
CAM TOOL DELETE n          CAM TOOLS            (the library with feeds and speeds)
```

**Defaults by tool type:**

| Type | Flutes | Flute length | Other |
| --- | --- | --- | --- |
| Flat or bull-nose end mill | 3 | 3 × diameter | |
| Ball end mill | 2 | 3 × diameter | |
| Face mill | 5 | 6 mm | |
| Drill | 2 | 5 × diameter | 118° point |
| Spot drill, chamfer mill | 2 | 3 × diameter | 90° |
| Tap | 3 | 3 × diameter | ISO coarse pitch (M6: 1.0 mm) |

*Manufacture → Add a Starter Tool Library* loads ten common tools.

**Speeds and feeds:**

- Spindle speed: `rpm = Vc × 1000 / (π D)`, clamped to the machine's maximum.
- Feed: `rpm × flutes × chip load`.
- Plunge and ramp feed: 30% of the feed.
- Light stepovers thin the chip, so the feed rises by `D / (2 √(ae (D − ae)))` (at most 3×).
  This is how a 10% stepover in aluminium runs about 1.7× faster.
- Drills feed per turn. Taps feed exactly one pitch per turn.
- `RPM`, `FEED` and `PLUNGE` on an operation override the values from the tool and material.

## Operations

Each operation is added, generated and reported straight away:

```text
Added 2. Rough with T1  8 mm flat end mill: 7190 moves, 19.19 m cutting, 10 min 39 s, S11930 F2337 (plunge 701) mm/min
```

If no `TOOL` is given, the operation uses the first suitable tool in the library.

| Command | What it does |
| --- | --- |
| `CAM FACE` | Zig-zag passes over the whole stock, from the stock top down to the part's top. Default stepover 70%, depth steps up to 1 mm. |
| `CAM ROUGH` | 3D roughing (see below). |
| `CAM POCKET` | Roughing of the regions the part closes in on every side. |
| `CAM CONTOUR` | The profile at each depth down to `BOTTOM`. See below. |
| `CAM PARALLEL` | Passes across the part (`ANGLE` from X), following the surface. See below. |
| `CAM WATERLINE` | Constant-height loops around the part, level by level. See below. |
| `CAM DRILL` / `SPOT` / `PECK` / `TAP` | Drilling at the part's holes that fit the tool. See below. |
| `CAM BORE` | Helical interpolation of holes wider than the end mill. See below. |

**`CAM ROUGH`** clears the part at each depth:

- **Pattern:** rings a stepover apart, from the middle of each region outwards (default 40% of
  the diameter, taken from a distance field). `RASTER` gives zig-zag instead. The region's
  edge is always finished at that depth.
- **Depths:** extra depths are added at the part's flat faces, so floors come out exact.
- **Entries:** the tool ramps in at 3° along the ring. Tiny regions get a plunge.
- **Links:** inside a region the tool stays down when the link is clear. Otherwise it lifts
  only as high as the part under the way needs.
- **Depth of cut:** from the material (half the diameter in aluminium, a quarter in steel).
  `LEAVE` sets the stock to leave.

**`CAM CONTOUR`** cuts the profile:

- `OUTSIDE` (the default) cuts the outside profile, `INSIDE` cuts the walls of pockets and
  holes, and `BOTH` cuts all of them.
- Climb milling is the default (`CONVENTIONAL` reverses it).
- Lead in and out is a quarter arc (`LEAD r`, default half the radius) on the free side.
- `TABS n` (with `TABWIDTH` and `TABHEIGHT`) adds holding tabs on the last depths.

**`CAM PARALLEL`** finishing:

- `SCALLOP h` sets the stepover from the cusp height left. With a ball end mill it is
  `2 √(h (2r − h))`.
- The passes are sampled densely. Points are added wherever a straight move would cut into the
  part, then dropped where they lie in a straight line.
- `SLOPE a b` or `SHALLOW` (0–50°) limits it to the gentler surfaces.

**`CAM WATERLINE`** finishing:

- Each level's loops are found exactly, like a contour.
- `STEEP` (45–90°) limits it to the walls, so it pairs with a shallow parallel pass.

**`CAM DRILL`** and its cycles:

- `CYCLE spot|drill|peck|chipbreak|tap|bore`, with `PECK q` and `DWELL s`.
- Depths:
  - Through holes are drilled past the bottom by the drill's point plus 0.5 mm.
  - Spot drills cut a spot just wider than the hole.
  - Taps stop two pitches short of a blind hole's bottom.
- `AT x,y x,y … [DEPTH d | THROUGH] [DIAMETER d]` gives the holes by hand.

**`CAM BORE`** cuts holes wider than the end mill:

- The tool goes down a helix at a 4.5° pitch, makes a flat turn at the bottom, then moves to
  the middle and out.
- The radius is checked against the part with the exact cutter test, so a faceted hole is not
  gouged.

**Options common to all the operations:**

`TOOL n`, `STEPDOWN`, `STEPOVER mm|n%`, `LEAVE`, `TOP z`, `BOTTOM z`, `DEPTH d` (below the
stock top), `ANGLE`, `NAME "x"`, `RPM`, `FEED`, `PLUNGE`.

**Holes:** `CAM HOLES` lists the part's round vertical holes. For each it gives the size
(fitted to a circle), the centre, the top and bottom, and whether it goes through.

## The job

```text
CAM LIST       CAM GENERATE       CAM DELETE n       CAM CLEAR       CAM SHOW | HIDE
```

- **Saving:** the setup, tools and operations are saved in the project. The toolpaths are
  remade when needed.
- **Staleness:** a part that changes or moves after its paths were made gets them remade by
  the next CAM command.
- **Display:** the toolpaths show in the 3D view. Cuts are in each operation's colour, plunges
  are yellow and rapids are thin red.

## Simulation

```text
CAM SIMULATE [SHOW]
```

The stock is modelled as a fine height field: a grid of an eighth of the smallest tool's
radius, up to 2.5 million points (coarser when one tool would cover more than about 250 000).
Each move cuts it with the tool's real shape (flat, ball, bull nose or drill point), exactly
along the move: the lowest the cutting edge reaches over each point.

The report gives:

- **Volume removed.**
- **Stock left on the part:** the most and the average.
- **Gouges:** points cut into the part by more than twice the tolerance, measured square to
  the surface. A path may sit the tolerance sideways into a wall, so the part's height is taken
  as the lowest within that reach of each point.
- **Rapid moves through stock.**
- **Shank hits:** stock above a tool's flute length while it is cutting.
- **Cycle time:** the feeds, the rapid rate and the tool changes.

`SHOW` adds the cut stock as a body, *Stock (simulated)*, and hides the part.

## Post processors

```text
CAM POST file.nc [FANUC | HAAS | GRBL | LINUXCNC]
```

| | Fanuc | Haas | LinuxCNC | GRBL |
| --- | --- | --- | --- | --- |
| Frame | `%`, `O1000 (NAME)`, `%` | the same | `%` | none |
| Units / modes | `G21 G17 G40 G49 G80 G90 G94` | `G00 G17 G40 G49 G80 G90`, `G21` | the same plus `G64 P` tolerance | `G21 G17 G90 G94` |
| Tool change | `T1 M06`, `G43 H01` | the same | `T1 M6`, `G43 H1` | `M0 (Change to T2 …)`, then touch off |
| Drilling | `G81 G82 G83 G73 G84 G85` with `G98`, `G80`; `M29` rigid tap | the same, no `M29` | the same; taps with `G33.1` | expanded to moves; tapping refused |
| End | `G91 G28 Z0.`, `M30` | `G53 Z0.`, `M30` | `G53 G0 Z0`, `M2` | `M30` |

**Coordinates and words:**

- Coordinates are relative to the work zero.
- Only the words that change are written.
- Fanuc and Haas numbers keep their decimal point (`25.`).

**Arcs:**

- Arcs (`G2`/`G3` with `I J`, and helical arcs with `Z`) are fitted to runs of cuts that lie on
  a circle within the tolerance.
- They make up rounded corners, bores and lead arcs.

**Tool length:** a fresh tool always moves over in X and Y first. It goes down in Z only after
`G43` puts its length offset on, so no Z move happens without it.

## Over MCP

The `cam` tool has these operations:

- `setup`, `tool`;
- `face`, `rough`, `pocket`, `contour`, `parallel`, `waterline`, `drill`, `bore`;
- `holes`, `list`, `generate`, `simulate`, `post`, `delete`, `clear`.

Its parameters are listed in the tool's schema.

```json
{"operation": "setup", "body": "Bracket", "stock_margin": 3, "material": "aluminium", "post": "haas"}
{"operation": "tool", "number": 1, "type": "flat", "diameter": 6}
{"operation": "rough", "leave": 0.3, "stepover_percent": 35}
{"operation": "simulate", "show": true}
{"operation": "post", "path": "bracket.nc"}
```

## Not yet

These are not modelled yet:

- **Clearing:** trochoidal "dynamic" clearing (the clearing here holds a constant stepover,
  with chip thinning and ramped entries); rest machining from the previous operation's stock.
- **Machines:** 4- and 5-axis; lathes.
- **Collisions:** the tool holder in the collision check (the check covers the flute length
  only).

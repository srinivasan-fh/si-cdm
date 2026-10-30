# Getting started

SI CDM designs parts and makes them: 3D models with sketches and features, 2D drawings,
assemblies, and the programs that machine them (milling and turning) or print them. This
page takes you through the window and a first part, from a sketch to G-code.

## The window

- **Menu bar** (top): *File*, *View*, *Design*, *Drawing*, *Assembly*, *Manufacture*, *AI*,
  *Plugins* and *Help*.
- **Options bar** (below it): the active tool's options on the left; on the right the
  *2D Drawing*, *3D Model* and *Simulation* windows, the display mode, perspective, *Fit* and
  the grid.
- **Tool rails** on all four sides of the canvas. Hover over a tool for its name and key;
  right-click or hold one with a corner mark for the rest of its kind.
  - **Left:** select, move and view tools, solids, features, surfaces and mesh tools.
  - **Top (*Sketch*):** point, line, arc, circle, rectangle and spline, then trim, extend and
    offset.
  - **Right:** sculpting brushes, cuts and booleans, direct editing, and armour.
  - **Bottom (*View*):** home, fit and zoom; top, front, right and isometric views; shaded,
    shaded with edges, wireframe and hidden-line display; the section view, grid, snapping to the
    grid and to points, and perspective or orthographic.
- **Panel** (beside the rail): *Model* (`M`) lists the bodies, sketches, mates and images;
  *Library* (`K`) has ready-made parts; *CAM* (`J`) runs the machining jobs.
- **Canvas**: the model. The view sphere (bottom right) turns the view: click on it for the
  view nearest that spot (the 26 of a cube's faces, edges and corners: *Top*, *Front*, *Right*
  are named on it), or drag it to orbit.
- **Command bar** (bottom): everything the menus and tools do is also a command you can
  type, such as `BOX 40 20 10`. `HELP` lists them, `HELP BOX` explains one, and Tab completes
  a name. The log above it shows what each action did.
- **Status bar**: the cursor's position, the units and the counts of what is in the file.

Every file opens in its own tab above the canvas. *File → Undo* (`Ctrl+Z`) and *Redo*
(`Ctrl+Shift+Z`) take back model edits and machining changes alike.

## Moving around

| Do this | To |
| --- | --- |
| Left or right drag on empty space | Orbit (3D), pan (2D) |
| Middle drag, or Shift + left drag | Pan |
| Scroll | Zoom at the cursor |
| `F` | Fit everything in view |
| `1` `3` `7` `0` | Front, right, top and isometric views |
| `P` | Perspective or orthographic |
| `W` | Shaded, shaded with edges, wireframe, hidden line |
| `Tab` | Switch between the 2D drawing and the 3D model |

*Help → Keyboard and Mouse* lists every key.

## A first part

1. **Units:** *View → Units* sets millimetres, inches and the others. Values you type take
   these units unless you write one (`20mm`, `0.75in`).
2. **Sketch:** press `R` (rectangle) and click two corners on the grid, or type
   `RECT -30,-20 @60,40` in the command bar. `C` draws circles and `L` lines; sketches lie on
   the drawing plane (*Top* by default).
3. **Extrude:** press `E`, click inside the rectangle, and drag up, or type the depth
   (`15`) and press Enter. The sketch becomes a solid body.
4. **Cut a hole:** pick the hole tool on the rail and click the top face, or type
   `HOLE "Extrude 1" AT 20,0,15 DIAMETER 6 THROUGH`.
5. **Round the edges:** select the body and use *Design → Fillet (Round Edges)*, or type
   `FILLET RADIUS 2`.
6. **Save:** *File → Save* (`Ctrl+S`) writes a `.sicdm` project. Your work is also saved
   every minute in the background, and after a crash SI CDM offers it back.

The [CAD guide](cad.md) covers sketches, constraints, features, patterns and the feature
history; [STEP and IGES](step.md) covers exchanging models with other CAD systems, and
[mesh formats](mesh-formats.md) the 3MF, AMF, PLY, OFF, glTF and VRML files that printers,
scanners and viewers use.

## A drawing

Press `Tab` for the 2D window, then *Drawing → New Drawing On* a sheet size. Add the
standard views, dimensions, tolerances and GD&T, and export to PDF, DXF or SVG. See the
[drawing guide](drawing.md).

## Machining it

1. **Open the CAM panel:** press `J`.
2. **Set up:** select the part and press *Set Up the Selected Part*. The stock is the
   part's box plus 2 mm; pick the material and the post processor (your machine's control).
3. **Tools:** *Tools...* opens the tool library. *Add a Starter Set* gives ten common tools,
   or load your own catalogue (CSV) or Fusion 360 library.
4. **Machine:** *Machine...* sets the machine's travel, rotary axes, spindle and feeds, from a
   preset or your own figures.
5. **Operations:** *+ Add Operation*, for example *Face*, *3D Rough*, *Parallel Finish* and
   *Drill Holes*. Each is made at once and drawn in the view. Pick one in the list to
   change its tool, depths, stock to leave or feeds, then press *Apply*.
6. **Simulate:** *Simulate* cuts the stock and shows it, and reports gouges, rapid moves
   through stock and holder collisions.
7. **Post:** *Post G-code...* writes the program. It is read back the way the control would
   run it and checked against the toolpath and the machine before it is saved.

Turning works the same way in the lower half of the CAM panel. See the
[CAM guide](cam.md) and the [lathe guide](lathe.md).

> Before running any program on a machine, check it: run it in the air first (above the
> part, or with the work offset raised), then on soft material. SI CDM's simulation and
> read-back catch many mistakes, but not every one a real machine can make.

## Samples

*Help → Sample Projects* builds finished examples: a part with a drawing, a milling job, a
lathe job and a sculpted head. Open one and look through its panels.

## Settings

*File → Preferences* (`Ctrl+,`) sets the units new projects start in, how often unsaved work
is autosaved, the material, post and machine a CAM job starts with, and the update check.
*File → Open Recent* reopens the last ten projects. See
[preferences, recent files and updates](preferences.md).

## Getting help

- *Help → User Guide* (`F1`) opens these pages inside SI CDM; search them from its box.
- `HELP` in the command bar lists every command; `HELP name` explains one.
- If SI CDM stops unexpectedly, it writes a crash report (`CRASHES` lists them) and offers
  your unsaved work back the next time it starts. Send the report with a note of what you
  were doing.

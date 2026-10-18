# SI CDM

A high-performance CAD / CAM desktop application written in C++20.

![3D Model window](docs/landing.png)

![2D Drawing window](docs/drawing-2d.png)

## Run

Requires CMake 3.24+ (`brew install cmake`) and, on macOS, Xcode 15+.

```bash
./run.sh            # build (Release) and launch the app (also builds sicdm-mcp)
./run.sh debug      # build (Debug) and launch
./run.sh test       # build and run the unit tests
./run.sh xcode      # generate build/xcode/SICDM.xcodeproj and open it in Xcode (macOS)
./run.sh clean      # delete the build folder
```

In Xcode, select the **sicdm** scheme and press ⌘R.

## Files in tabs

Every open file has its own tab at the top of the canvas, AutoCAD style: `☰` (file menu and the list
of open files), one tab per file (`*` = unsaved, `×` or middle-click closes it) and `+` for a new file.
Each file keeps its own model, 2D / 3D window, cameras and selection, so switching tabs puts you back
exactly where you were. `Ctrl/⌘ N` and `+` open a new tab, Open opens the project in its own tab (or
shows it if it is already open; the untouched demo part is replaced), `Ctrl/⌘ W` closes the tab —
asking to save first if needed — and `Ctrl + Tab` / `Ctrl + Shift + Tab` cycle through the files. On the
command line, `FILES` lists the tabs, `FILES 2` or `FILES part` switches, `CLOSE [DISCARD]` closes.

## View windows

The top bar of the canvas switches between two windows of the active file (also `Tab`):

- **3D Model** – perspective / orthographic model view with Shaded, Shaded + Edges, Wireframe and
  Hidden Line display.
- **2D Drawing** – exact orthographic plan view of the Top, Front or Right plane, rendered as a
  hidden-line drawing with mm rulers, a plane grid and cursor coordinates. Drag pans, the wheel
  zooms at the cursor; the view never tilts.

Each window keeps its own camera, so switching back and forth never loses your view.

**ViewCube** (bottom right): click any face, edge or corner for one of 26 views, drag it to orbit,
use Home, the roll arrows (top right: spin the view 90° around the viewing direction, Shift for 15°)
and the ◀ ▶ arrows by the projection badge (turn 90° around Z). The badge shows the roll angle;
right-click for more views, including Level View to reset the roll. Picking a face, edge or corner
always comes back level. In the 2D window its faces pick
the drawing plane; edges and corners open the 3D window at that angle.

## Tools (left rail)

Photoshop-style tool rail. `<<` / `>>` switches between two columns and one.
The tree button at the top of the rail (or `M`) shows the **Model panel**, docked beside the rail:
bodies, sketches and properties of the selection. It is hidden by default; drag its right edge to resize.
Slots with a small corner triangle hold several tools: right-click or hold to open the flyout.

- **Navigate & select:** Move (V), Select (S), Orbit (O), Pan (H), Zoom (Z)
- **2D sketch:** Point, Line (L), Polyline, Rectangle (R), Circle (C) on the active plane, in either
  window. Grid and endpoint snapping, Shift locks horizontal / vertical, live dimensions,
  Enter / double-click finishes a polyline, Esc cancels.
- **3D solids:** Block (B), Wedge, Pyramid, Prism, Cylinder (Y), Cone, Sphere, Torus.
  Pick one and click the grid or a face to place it.
- **Extrude (E):** turns a closed sketch profile (rectangle, circle or closed polyline, concave shapes
  included) into a solid along its plane's normal. Click the profile (its outline or anywhere inside),
  drag to set the depth with a live preview — or type it in the options bar or command line — then
  click or press Enter. Options: symmetric (both sides of the plane), keep sketch visible. Negative
  depths go the other way. The profile is hidden after extruding (show it again in the Model panel).
  Extrusions are parametric: they save and reopen as features.
- **Planned:** the remaining 2D tools, other form features (revolve, sweep, loft…), surfaces, Class-A, generative and mesh tools
  are in the rail with descriptions and will be enabled in later milestones.

Shift + a tool key cycles through that slot's tools.

## Command line

The full-width command line above the status bar runs every operation by typing, AutoCAD style.
`Space` focuses it, `Tab` completes a command name, `↑` / `↓` recall earlier input, and the ▲ button
shows the history. Type `HELP` for the full list.

```text
LINE 0,0 @50,0 @0,30          RECT 0,0 @40,20          CIRCLE 10,10 5        PLINE 0,0 10,0 10,10 CLOSE
BOX 40 30 20 @10,0,5          CYLINDER 10 30           SPHERE 8              TORUS 20 4
EXTRUDE 15                    EXTRUDE "Circle 2" 20 SYMMETRIC KEEP
SELECT Block 1   MOVE 0,0,10   ERASE        VIEW FRONT   2D RIGHT   3D   ROLL 90   DISPLAY HIDDEN
IMPORT "plans/scan.jpg" WIDTH 150 @0,0      EXPORT part.stl      SAVE project.sicdm      OPEN project.sicdm
```

With a sketch tool active, typed coordinates place points exactly: `x,y`, `@dx,dy` (relative) or
`@length<angle` (polar); for a circle, a single number is the radius; `Enter` finishes a polyline.

## Files

| Menu (shortcut) | Formats |
| --- | --- |
| **New / Open / Save / Save As** (Ctrl/⌘ N, O, S, Shift+S) | `.sicdm` project (JSON): parametric solids, meshes, sketches, reference images |
| **Import** (Ctrl/⌘ I) | `.stl` `.obj` → body · `.dxf` → sketch curves on the active plane · `.png .jpg .bmp .tga .gif .psd .hdr .pnm` → reference image |
| **Export** (Ctrl/⌘ E) | `.stl` `.obj` (visible bodies) · `.dxf` `.svg` (sketch curves of the active plane) |

Unsaved changes are tracked (`*` in the tab and title); closing a tab or quitting asks before discarding them.

**Reference images** (scanned drawings, photos, blueprints) lie on the active sketch plane and show in
both windows: under the sketch in 2D for tracing, in place in 3D. Select one to set its width (mm),
centre and opacity in the Model panel, or drag it with the Move tool.

## AI agents (MCP)

SI CDM includes a [Model Context Protocol](https://modelcontextprotocol.io) server so AI agents can read
and edit the 2D and 3D model. Tools: `get_scene`, `create_sketch`, `create_solid`, `extrude`, `modify`, `delete`,
`set_view`, `run_command`, `list_commands`, `new_project`, `open_project`, `save_project`,
`import_file`, `export_file`.

- **Live app (HTTP):** enable *AI → MCP Server* or start with `--mcp-port 8765`, then
  `claude mcp add --transport http si-cdm http://127.0.0.1:8765/mcp`. The agent drives the open
  window (views, 2D/3D switch and file tabs included: `run_command` with `FILES`, `CLOSE`). Listens on localhost only.
- **Headless (stdio):** `sicdm-mcp [project.sicdm]` works on files without a window:
  `claude mcp add si-cdm-files -- /path/to/build/release/sicdm-mcp`.

*AI → Connect an AI Agent…* shows ready-to-copy settings.

## Controls

- `Tab`: switch 2D Drawing / 3D Model window
- `Ctrl + Tab` / `Ctrl + Shift + Tab`: next / previous file tab; `Ctrl/⌘ W`: close the file tab
- `Space`: command line
- Right drag (or left drag on empty space): orbit in 3D, pan in 2D
- Middle drag / Shift + left drag: pan
- Scroll: zoom at the cursor
- Delete: remove the selected body, sketch curve or image
- `F`: fit to view
- `1` / `3` / `7` / `0`: Front / Right / Top / Isometric (in 2D: pick the drawing plane)

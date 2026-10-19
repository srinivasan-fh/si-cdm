<img src="resources/icon/sicdm_256.png" width="96" alt="SI CDM icon" align="right">

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

## Launch: icon and splash screen

![Launch splash](docs/splash.png)

The app starts with a cinematic splash: a six-wheeled rover crossing a Martian plain at golden hour,
rendered live on the GPU (ray-marched terrain with dunes, boulders and wheel ruts, a rover with
rocker-bogie suspension, mast camera, arm and RTG, low sun with soft shadows, dusty aerial
perspective, filmic grade, 2.39:1 letterbox). The bottom bar shows the real loading steps (graphics,
fonts, shaders, part library, workspace, MCP server, the file being opened) with a progress line;
the main window appears when loading is done. Click or press Esc to skip it.

```bash
sicdm --no-splash            # start straight into the workbench
sicdm --splash-seconds 6     # keep the splash up longer (default 3.2 s minimum)
```

The splash renders at a resolution adapted to the GPU, so it stays smooth on integrated graphics.
`--screenshot` runs never show it.

The app icon (a machined blue part over the rim of Mars, ringed by an orbit, on a deep-space squircle) is drawn by
`tools/make_icon.py` (Python + Pillow + NumPy) into `resources/icon/*.png` and
`resources/macos/SICDM.icns`. The macOS bundle uses the `.icns` for the Dock and Finder; on
Windows and Linux the PNGs are compiled into the app and set as the window icon.

## Interface

Dark graphite workbench in the style of AutoCAD, Maya and Photoshop: Inter for the interface,
JetBrains Mono for the command line and numeric readouts (both SIL OFL, embedded in the app from
`resources/fonts`), 22 pt controls on an 8 pt spacing grid, one accent blue for selection and the
active window. Text is rasterised at the display's pixel density, so it is sharp on Retina screens.
Colours, fonts and spacing live in `src/ui/Theme.hpp`.

## Files in tabs

Every open file has its own tab at the top of the canvas, AutoCAD style: `☰` (file menu and the list
of open files), one tab per file (`*` = unsaved, `×` or middle-click closes it) and `+` for a new file.
Each file keeps its own model, 2D / 3D window, cameras and selection, so switching tabs puts you back
exactly where you were. `Ctrl/⌘ N` and `+` open a new tab, Open opens the project in its own tab (or
shows it if it is already open; the untouched demo part is replaced), `Ctrl/⌘ W` closes the tab —
asking to save first if needed — and `Ctrl + Tab` / `Ctrl + Shift + Tab` cycle through the files. On the
command line, `FILES` lists the tabs, `FILES 2` or `FILES part` switches, `CLOSE [DISCARD]` closes.

## View windows

The top bar of the canvas switches between three windows of the active file (also `Tab`; Simulation is described below):

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

## Simulation window (motion control)

![Simulation window](docs/simulation.png)

The third window, **Simulation** (top bar, *View → Simulation Window* or `Shift + Tab`), plays motions
of the 3D model without changing it — Stop puts every body back where it was modelled.

- **Circular motion:** rotation about an axis (X / Y / Z or any direction) through a point, in deg/s
  (or rpm). The axis point defaults to the centre of the moved bodies, so a wheel spins about its hub;
  *Each body about its own centre* (`EACH`) lets one motion spin all the wheels of a vehicle.
- **Linear motion:** travel along a direction in mm/s.
- **Rolling drive:** a linear motion can *roll with* a circular one (`FOLLOW`): travel = wheel angle × radius
  (default: the wheel's size), so the wheels drive the vehicle — stop or reverse them and it stops or backs up.
- **Range & mode:** an optional sweep (deg) or stroke (mm) with Continuous, Loop, Oscillate (back and
  forth) or Once; a start delay; negative speeds reverse. Each motion can be switched on and off.
- **Chained motions:** a body in several motions gets them in list order, the first outermost — e.g. a
  linear *Drive* on all bodies listed before a circular *Wheel spin* on the wheels carries the spinning
  wheels along.
- **Playback** (options bar): rewind, play / pause (`Space`), stop, a timeline to scrub (`←` / `→` step
  1/30 s, `Home` rewinds), speed 0.1×–10× and loop. The length follows the motions or is set by hand.
- **Real-time axes:** live X / Y / Z triads on the moving parts and the selection, the rotation axes
  with a turning arrow and travel arrows with live values, optional paths, and a digital readout of
  the selected body — position X Y Z, rotation A B C and their velocities.
- **Motion Study panel** (right): add motions for the selected body (or all), edit every parameter
  live, add / remove bodies.

Motions are saved in the project. On the command line:

```text
MOTION LINEAR Drive DIR X SPEED 20 STROKE 200 OSC BODIES ALL
MOTION CIRCLE "Wheel spin" AXIS Y RPM 30 EACH BODIES "Wheel 1" "Wheel 2" "Wheel 3"
MOTION LINEAR Drive FOLLOW "Wheel spin"        # the body rolls with the wheels (RADIUS r to override)
MOTION CIRCLE Turntable AXIS Z AT 0,0,0 SPEED -45 RANGE 90 ONCE START 2
MOTION   MOTION OFF Drive   MOTION DELETE Drive
SIM PLAY   SIM PAUSE   SIM STOP   SIM TIME 2.5   SIM SPEED 2   SIM DURATION 12   SIM AXES OFF   SIM AT 3
```

`SIM AT t` (also headless) reports every motion value and moving body position at time t.

## Library: motors and gears

![Library](docs/library.png)

The gear button at the top of the tool rail (or `K`) opens the **Library** page of the docked panel:
search, pick a part, set its parameters and click **Insert** (or double-click a part). Parts are
parametric: they save as parameters and rebuild exactly when the project opens.

| Category | Parts |
| --- | --- |
| Motors | **DC motor** (28 mm can, 3 mm shaft), **Stepper motor** (NEMA 17, 1.8° steps), **Servo motor** (standard servo, two-arm horn), **Electric car motor (Foot)** (foot-mounted traction motor: ribbed end shield, terminal box, keyed hollow-end shaft), **Electric car motor (Flange)** (ribbed flange-mount EV motor, lifting eyes, splined shaft) |
| Gears | **Spur**, **Helical**, **Herringbone**, **Bevel**, **Internal ring**, **Worm**, **Worm wheel**: involute teeth, 20° pressure angle, module / teeth / face width / bore |
| Linear gears | **Gear rack** |
| Wheels & tyres | **Car wheel** (alloy rim, spokes, lug nuts, tyre), **Robot wheel**, **Caster wheel** (swivel plate, fork), **Omni wheel**, **Mecanum wheel**, **Tyre** (road / off-road / slick tread) |
| Chains & belts | **Sprocket** (ANSI 40 by default), **Steel roller chain** (plates, pins, rollers), **Chain drive**, **Pulley** (V-groove / timing GT2 / flat), **Rubber belt** (flat / V / timing), **Belt drive**, **O-ring / rubber band** |
| Gear trains | **Gear pair**, **Rack and pinion**, **Planetary set** (sun, 3 planets, carrier, ring), **Worm drive**: meshed, with linked motions |

- **Motors** are two bodies, a fixed housing and a rotor (shaft with coupler, or servo horn), and come
  with a **360° circular motion** on the rotor (speed in rpm, Loop 0–360°). Steppers move in 1.8°
  steps; a servo with a sweep under 360° swings back and forth.
- **Wheels** stand on the ground with the axle along Y and spin (rpm, 0 = no motion) so that positive
  speeds roll toward +X; omni and mecanum rollers follow the rolling circle exactly. **Chain and belt
  drives** turn the driven sprocket / pulley at the right ratio (same direction), and the **chain or
  belt travels along its loop** with the driver (travel = driver angle × pitch radius): links, rollers
  and timing teeth run round the sprockets / pulleys; smooth belts carry printed marks so their motion
  shows. Any linear motion can do this for a chain or belt body with *Travel along the chain / belt
  loop* in the Motion Study panel, or `MOTION LINEAR name PATH FOLLOW driver RADIUS r BODIES belt`.
- **Electric car motors** lie on their side: the shaft and the 360° drive run along +X at the shaft
  height (170 / 120 mm at size 1). Shaft end: plain or with a flange coupling (foot motor), splined
  or plain (flange motor).
- **Shafts:** DC, stepper and electric car motors have *Shafts* (None / Single / Dual), *Shaft length*, *Rear shaft*
  length (dual), *Shaft diameter* and *Shaft end* (plain or with a coupler). Servos choose a *Horn*
  (single arm, double arm, cross, disc). Set them before inserting, or afterwards: select the motor and
  edit *Properties → Shaft* (live), or `PARAM "Stepper Motor 1" SHAFTS DUAL REAR_LENGTH 20`. Gears and
  racks are editable the same way (teeth, module, width, bore...).
- **Scalable:** every motor (and any body) has a **Size** factor, set in the Library before inserting,
  or afterwards in *Properties → Scale* or with `SCALE`. A component scales as a whole: its bodies,
  their spacing and its motions.
- **Gear trains** drive each other: a circular motion can be *Driven by* another one at a ratio
  (−z₁/z₂ for an external mesh), chained as far as you like (motor → gear → gear); a linear motion rolls
  with a circular one (rack = pinion angle × pitch radius). Set it in the Motion Study panel or with
  `MOTION ... FOLLOW name RATIO r`.

```text
PART                              (lists the library)
PART DCMOTOR SIZE 1.5 RPM 60      PART STEPPER @100,0,0 RPM 30 STEP 1.8      PART SERVO NAME Pan SWEEP 180
PART STEPPER SHAFTS DUAL SHAFT_LENGTH 30 REAR_LENGTH 20 SHAFT_DIA 8 FITTING PLAIN      PART SERVO HORN DISC
PART CARWHEEL DIAMETER 650 RIM_DIA 432 SPOKES 5      PART TYRE TREAD OFF-ROAD      PART MECANUM DIAMETER 100
PART CHAINDRIVE TEETH1 12 TEETH2 36 CENTER 250      PART BELTDRIVE TYPE TIMING DIA1 40 DIA2 80      PART ORING INNER_DIA 30
PART EVMOTOR SIZE 0.8 SHAFT_LENGTH 150      PART EVFLANGE SHAFTS DUAL FITTING SPLINED      PARAM "DC Motor 1" SHAFTS NONE      PARAM "Spur Gear 1" TEETH 32      PARAM "Stepper Motor 1"   (lists them)
PART SPUR MODULE 2 TEETH 24 WIDTH 10 BORE 4      PART HELICAL TEETH 30 HELIX 20      PART RACK TEETH 40
PART GEARPAIR TEETH1 16 TEETH2 48      PART PLANETARY SUN_TEETH 18 PLANET_TEETH 18      PART WORMDRIVE
SCALE "DC Motor 1" 2      SCALE "Gear Pair 1 Driver" TO 1
MOTION CIRCLE Output FOLLOW "DC Motor 1 Drive" RATIO -0.25 BODIES "Spur Gear 1"
```

## Tools (left rail)

Photoshop-style tool rail. `<<` / `>>` switches between two columns and one.
The tree button at the top of the rail (or `M`) shows the **Model panel**, docked beside the rail (its
*Library* page holds motors and gears, see above):
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
and edit the 2D and 3D model and its motion. Tools: `get_scene`, `create_sketch`, `create_solid`, `extrude`, `modify`, `delete`,
`create_motion`, `delete_motion`, `simulate` (play / pause / seek, or body poses at a time — headless too),
`insert_part`, `list_parts` (library motors, gears and gear trains; `modify` takes a `scale`),
`set_view`, `run_command`, `list_commands`, `new_project`, `open_project`, `save_project`,
`import_file`, `export_file`.

- **Live app (HTTP):** enable *AI → MCP Server* or start with `--mcp-port 8765`, then
  `claude mcp add --transport http si-cdm http://127.0.0.1:8765/mcp`. The agent drives the open
  window (views, 2D / 3D / Simulation switch and file tabs included: `run_command` with `FILES`, `CLOSE`). Listens on localhost only.
- **Headless (stdio):** `sicdm-mcp [project.sicdm]` works on files without a window:
  `claude mcp add si-cdm-files -- /path/to/build/release/sicdm-mcp`.

*AI → Connect an AI Agent…* shows ready-to-copy settings.

## Controls

- `Tab`: switch 2D Drawing / 3D Model window; `Shift + Tab`: Simulation window
- Simulation: `Space` play / pause, `←` / `→` step, `Home` rewind
- `Ctrl + Tab` / `Ctrl + Shift + Tab`: next / previous file tab; `Ctrl/⌘ W`: close the file tab
- `Space`: command line
- Right drag (or left drag on empty space): orbit in 3D, pan in 2D
- Middle drag / Shift + left drag: pan
- Scroll: zoom at the cursor
- Delete: remove the selected body, sketch curve or image
- `F`: fit to view
- `1` / `3` / `7` / `0`: Front / Right / Top / Isometric (in 2D: pick the drawing plane)

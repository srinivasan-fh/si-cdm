# SI CDM

A high-performance CAD / CAM desktop application written in C++20.

![Landing page](docs/landing.png)

## Run

Requires CMake 3.24+ (`brew install cmake`) and, on macOS, Xcode 15+.

```bash
./run.sh            # build (Release) and launch the app
./run.sh debug      # build (Debug) and launch
./run.sh test       # build and run the unit tests
./run.sh xcode      # generate build/xcode/SICDM.xcodeproj and open it in Xcode (macOS)
./run.sh clean      # delete the build folder
```

In Xcode, select the **sicdm** scheme and press ⌘R.

## Tools (left rail)

Photoshop-style tool rail. `<<` / `>>` switches between two columns and one.
Slots with a small corner triangle hold several tools: right-click or hold to open the flyout.

- **Navigate & select:** Move (V), Select (S), Orbit (O), Pan (H), Zoom (Z)
- **3D solids:** Block (B), Wedge, Pyramid, Prism, Cylinder (Y), Cone, Sphere, Torus.
  Pick one and click the grid or a face to place it.
- **Planned:** 2D sketches, form features, surfaces, Class-A, generative and mesh tools
  are in the rail with descriptions and will be enabled in later milestones.

Shift + a tool key cycles through that slot's tools.

## Controls

- Right drag (or left drag on empty space): orbit
- Middle drag / Shift + left drag: pan
- Scroll: zoom
- Delete: remove the selected body
- `F`: fit to view
- `1` / `3` / `7` / `0`: Front / Right / Top / Isometric

# SI CDM — CAD / CAM Workbench

A high-performance CAD and CAM desktop application written in modern C++ (C++20).
The project is CMake-based and generates a native **Xcode** project on macOS.

> **Milestone 1: landing page.** The app opens on an interactive 3D canvas with a
> demo part, a CAD-style orbit camera, standard views, an adaptive grid, a navigation
> triad, a model tree and a welcome panel.

![Landing page](docs/landing.png)

## Getting started on macOS (Xcode)

Requirements: macOS 12+, Xcode 15+, CMake 3.24+ (`brew install cmake`).

```bash
./scripts/generate-xcode.sh          # creates build/xcode/SICDM.xcodeproj and opens it
```

In Xcode select the **sicdm** scheme and press **Run** (⌘R). For profiling, set the scheme's
build configuration to *Release* (Product → Scheme → Edit Scheme).

From the command line:

```bash
cmake --preset xcode
cmake --build --preset xcode-release
ctest --preset xcode
open "build/xcode/Release/SICDM.app"
```

The Xcode preset builds a Universal binary (`arm64;x86_64`), Release uses `-O3` with ThinLTO,
and `NSSupportsAutomaticGraphicsSwitching` is off so dual-GPU Macs use the discrete GPU.

## Linux / other platforms

```bash
sudo apt install libgl-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libxkbcommon-dev
cmake --preset release && cmake --build --preset release -j && ctest --preset release
./build/release/sicdm
```

`--screenshot out.ppm` renders a few frames, saves the framebuffer and exits (used by CI);
`--size W H` sets the window size.

## Viewport controls

| Input | Action |
| --- | --- |
| Left drag | Orbit |
| Middle drag, right drag, Shift + left drag | Pan |
| Scroll | Zoom |
| `F` | Zoom to fit |
| `1` / `3` / `7` / `0` | Front / Right / Top / Isometric (Ctrl/⌘ for Back / Left / Bottom) |
| `P` | Perspective ↔ orthographic |
| `W` | Cycle Shaded / Shaded + Edges / Wireframe |
| `G` | Toggle grid |
| Click a triad axis | Look down that axis |

## Architecture

```
src/
  core/     Platform-independent kernel: math, orbit camera, geometry, scene.
            No windowing or GPU dependencies; unit tested headless.
  render/   OpenGL 4.1 core renderer: gradient background, shaded bodies,
            feature edges / wireframe, anti-aliased adaptive grid.
  ui/       Landing page UI (Dear ImGui): menu bar, view toolbar, model tree,
            welcome card, navigation triad, status bar.
  app/      GLFW window, input → camera, on-demand frame loop.
tests/      Dependency-free unit tests (ctest).
cmake/      Compiler flags and pinned third-party dependencies (FetchContent).
```

Design notes:

- **Z-up, right-handed, millimetres** — the usual CAD convention.
- **On-demand rendering.** The frame loop blocks in `glfwWaitEvents()` when nothing changes,
  so an idle viewport costs no CPU or GPU time.
- **OpenGL 4.1 core** is the highest version macOS supports, so one renderer runs on every
  platform. `render/` is the only module that touches GL, so a Metal backend
  (via metal-cpp, keeping the code in C++) can replace it without changing `core/`, `ui/` or `app/`.
- Dependencies are fetched and pinned at configure time: GLFW 3.4 and Dear ImGui 1.91.5.
  Everything else is first-party C++.

## Roadmap

1. ✅ Landing page: 3D canvas, camera, views, grid, model tree
2. Picking and selection highlighting in the viewport
3. Geometry kernel integration (B-rep, e.g. OpenCASCADE) and STEP / IGES / STL import
4. Sketcher and parametric features (extrude, revolve, fillet)
5. CAM: stock and setups, 2D contour / adaptive clearing, toolpath simulation, G-code post-processors
6. Metal renderer backend

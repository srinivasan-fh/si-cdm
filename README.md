# SI CDM

A high-performance CAD / CAM desktop application written in C++20.

![Landing page](docs/landing.png)

## Build on macOS (Xcode)

Requires Xcode 15+ and CMake 3.24+ (`brew install cmake`).

```bash
./scripts/generate-xcode.sh
```

This creates `build/xcode/SICDM.xcodeproj` and opens it. Select the **sicdm** scheme and press ⌘R.

## Build on Linux

```bash
cmake --preset release
cmake --build --preset release -j
./build/release/sicdm
```

## Controls

- Left drag: orbit
- Middle or right drag: pan
- Scroll: zoom
- `F`: fit to view
- `1` / `3` / `7` / `0`: Front / Right / Top / Isometric

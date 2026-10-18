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

## Controls

- Left drag: orbit
- Middle or right drag: pan
- Scroll: zoom
- `F`: fit to view
- `1` / `3` / `7` / `0`: Front / Right / Top / Isometric

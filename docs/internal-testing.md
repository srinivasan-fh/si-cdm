# Internal testing

This build is for testing inside the organisation. It is not yet a release for customers.
This page says what to test, what is known not to work yet, and how to report what you find.

## Installing

| Platform | Package | First start |
| --- | --- | --- |
| Windows 10 / 11 | `SICDM-<version>-windows-x64.exe` | Unsigned builds show *Windows protected your PC*: choose *More info → Run anyway*. |
| macOS 12 or later | `SICDM-<version>-macos.dmg`: drag **SI CDM** to Applications | Unsigned builds are refused at first: on macOS 15, *System Settings → Privacy & Security → Open Anyway*; on earlier versions, right-click the app and choose *Open*. |
| Ubuntu / Debian | `sudo apt install ./sicdm_<version>_amd64.deb` | Start *SI CDM* from the applications menu, or `sicdm`. |

Each package opens `.sicdm` projects with a double-click. *Help → About* shows the version:
put it in every report.

## What to test

Work through these, in your own way as well as the way the guide describes. Each one should
work without typing a command; the command bar is there if you prefer it.

1. **First start.** *Help → Getting Started*; then each of the *Help → Sample Projects*.
2. **A part from a sketch.** Sketch, extrude, cut a hole, round edges, pattern, mirror. Edit an
   early feature in the history and check the later ones follow. Undo and redo every step
   (`Ctrl+Z`, `Ctrl+Shift+Z`), including new bodies, moves and imports.
3. **Exchange.** Import STEP and IGES files from your own CAD system; export them back and
   open them there. Note any face that is missing, inside out or in the wrong place.
4. **A drawing.** Views, a section, dimensions, tolerances and GD&T; export to PDF and DXF and
   open the files in your usual tools.
5. **An assembly.** Mate two or three parts, check interference, print the parts list.
6. **Milling.** Set up a part on the CAM panel (`J`), add tools from the Tools window, set the
   machine, add operations, change them, reorder and suppress them, simulate, post. Compare the
   program with what your current CAM system writes for the same part.
7. **Turning.** The same on the lathe half of the panel.
8. **3D printing.** Check, orient and slice a part; open the G-code in your printer's software.
9. **Preferences.** *File → Preferences*: change the units for new projects to inches and make
   a part; set the CAM defaults; turn autosave off and on.
10. **Recovery.** With unsaved work open, end SI CDM from the task manager (or `kill`), start it
    again and restore the work.
11. **Licence.** Install the test licence file you were given (*Help → Licence*); remove it.

> **G-code is not yet proven on real machines.** Never run a posted program on a machine
> without your shop's usual checks: simulate it in the control, run it in the air above the
> part, then in soft material. Report any difference between SI CDM's simulation and the
> machine, however small.

## Known limitations

These are known; there is no need to report them.

- **English only.** The interface and the guide have no translations yet.
- **Undo** covers bodies, features, sculpting, sketches and the CAM and lathe jobs, but not
  mates and assembly changes, feature parameter edits (`PARAM`, `REGEN`), motions and poses, or
  drawing sheets.
- **Unsigned installers** unless the build was made with signing keys (see the warnings above).
- **File names** with characters outside your system's code page (for example Cyrillic or CJK
  names on a Western Windows) have not been tested on Windows.
- **Double-click to open** has been built and packaged on all three systems, but tried by hand
  only on Linux: please check it on Windows and macOS.
- **Updates** are checked only when your administrator has set a release feed; SI CDM never
  installs one by itself.
- **Very large models** (millions of triangles, assemblies of hundreds of parts) work but can
  be slow to cut, mesh and simulate.

## Reporting a problem

For each problem, send:

- the version (*Help → About*) and the operating system;
- what you did, step by step, what you expected and what happened instead;
- the project file (`.sicdm`) and any file you imported, if you can share them;
- a screenshot, if it shows the problem;
- after a crash: the crash report. SI CDM offers it at the next start, and `CRASHES` lists
  them (see [crash reports](licensing.md#crash-reports)). Reports stay on your computer
  until you send them.

The command log (above the command bar) shows what each action did: a screenshot of it
around the problem helps.

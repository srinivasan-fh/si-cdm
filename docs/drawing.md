# Drawings

SI-CDM makes 2D engineering drawings from the 3D model: the sheet a machinist or an
inspector reads. The views come from the model itself, so a drawing follows the part as it
changes.

![A drawing of a plate: front, top, right, section and isometric views](drawing-sheet.png)

```text
SHEET NEW A3                     front, top, right and isometric views, at a scale that fits
SHEET SECTION                    a section through the front view's middle
SHEET AUTODIM                    overall sizes and hole diameters
SHEET TITLE MATERIAL 6061-T6
SHEET EXPORT bracket.pdf         (or .svg, .dxf)
```

In the app, everything is in the **Drawing** menu: *New Drawing*, *Show Drawing*,
*Dimension Automatically*, *Section Through Front / Top*, *Add View*, *Scale*,
*Export...* and *Delete*. The drawing opens in its own window over the 3D view. The
drawing is saved in the project file.

## The sheet

- **Sizes:** A4, A3 (the default), A2, A1, A0, LETTER (ANSI A), TABLOID (ANSI B), ANSI C and
  ANSI D. Landscape by default, or `PORTRAIT`.
- **Projection:** first angle (ISO, the default) or third angle (ASME, `THIRD`). The
  projection symbol in the title block shows which.
- **Scale:** a standard scale (1:1, 1:2, 1:5, 2:1, 5:1, ...) chosen so the views fit,
  or the scale you give.
- **Border and title block:** a 10 mm border, and a 180 × 40 mm title block in the bottom right
  corner. It has the title, part number, material, drawn, checked, date, scale, units,
  projection, sheet, revision and size. Fill a field with `SHEET TITLE field text`. The fields
  are TITLE, PART, MATERIAL, DRAWN, CHECKED, DATE, REV, SHEET and COMPANY.

## Views

`SHEET NEW` lays out four views:

- the **front** view;
- the **top** view: below the front in first angle, above it in third angle;
- the **right** view: left of the front in first angle, right of it in third angle;
- an **isometric** view in the top right corner.

The front, top and right views line up with each other.

| Command | What it does |
| --- | --- |
| `SHEET NEW [bodies] [A3] [PORTRAIT] [THIRD] [SCALE 1:2]` | A new drawing of the named bodies (default: every visible body) |
| `SHEET VIEW ADD FRONT\|BACK\|TOP\|BOTTOM\|LEFT\|RIGHT\|ISO [AT x,y] [SCALE 2:1] [NOHIDDEN]` | Another view, placed where the projection puts it unless `AT` gives a place |
| `SHEET VIEW MOVE name x,y` | Moves a view's centre (sheet mm from the bottom left corner) |
| `SHEET VIEW SCALE name 2:1` | A view at its own scale, noted under it |
| `SHEET VIEW HIDDEN name ON\|OFF` | Hidden lines dashed, or left out |
| `SHEET VIEW DELETE name` | Removes the view and its dimensions |
| `SHEET SCALE 1:2` · `SHEET SIZE A2 [PORTRAIT]` · `SHEET PROJECTION FIRST\|THIRD` | Changes the sheet and lays the views out again |

When views run off the sheet, sit on the title block or overlap each other, the command says
so. A smaller scale, a larger sheet or `SHEET VIEW MOVE` fixes it.

**Which lines are drawn:**

- **Visible edges** are thick lines.
- **Hidden edges** are dashed. They are found by a depth test against the model itself.
- **Edges:**
  - On exact solids (STEP and IGES imports, primitives, extrusions, revolves), the boundaries
    between faces.
  - Smooth (tangent) edges are left out, except as thin lines in the isometric view, where they
    show the shape.
  - On meshes, creases sharper than 30°.
- **Silhouettes** of curved faces.
- **Centre lines:**
  - centre marks on holes and bosses seen end on;
  - centre lines along the axes of those seen from the side.

## Sections

```text
SHEET SECTION [view] [VERTICAL|HORIZONTAL] [AT position] [FLIP] [LABEL B]
```

**The cut line:**

- It crosses the view: up and down (`VERTICAL`, the default) or across (`HORIZONTAL`).
- `AT` places it in model mm along the view's horizontal or vertical axis. Without `AT`, it
  goes through the middle of the bodies.
- The parent view shows the cut line with a letter and arrows that point the way the section looks.
- `FLIP` looks the other way.

**The section view:**

- It shows what is left behind the cut.
- The cut faces are outlined and hatched at 45°.
- It is labelled SECTION A-A (B-B, ...).
- It is placed in a free space near its parent, on the side the viewer of the cut stands.
- Sections cut from a top or bottom view are turned the right way up.

## Dimensions

```text
SHEET AUTODIM [view]
SHEET DIM HORIZONTAL|VERTICAL|ALIGNED view x,y,z x,y,z [OFFSET mm] [TEXT text]
SHEET DIM DIAMETER|RADIUS view centre-x,y,z point-on-circle-x,y,z
SHEET DIM DELETE n · SHEET DIM CLEAR
```

**Tied to the model:** a dimension measures between model points, so it follows the model.

**What each kind measures:**

- **Horizontal** and **vertical:** the distance along the view's axes.
- **Aligned:** the length as the view shows it.
- **Diameters** and **radii:** read Ø and R.

**Placement:**

- `OFFSET` is how far the dimension line stands from the feature, in sheet mm. A negative
  offset puts it below or to the left.
- `TEXT` replaces the measured value, for example with a tolerance.

**`SHEET AUTODIM` adds:**

- each orthographic view's overall width and height, where another view doesn't already
  show them;
- the diameters of round holes and bosses seen end on.

Running it again adds nothing twice.

**In the drawing window:**

1. Pick *Dimension* in the tool list. A tolerance typed in its box goes on each new
   dimension.
2. Click two points. They snap to vertices and edge ends.
3. The dimension is horizontal, vertical or aligned, whichever the points suggest.

## Tolerances and GD&T

![Size tolerances, ISO fits, feature control frames, datums and a surface finish mark](drawing-gdt.png)

### Size tolerances

```text
SHEET TOL n 0.1                  80 ±0.1
SHEET TOL n +0.05 -0.1           12 with +0.05 over -0.1, stacked
SHEET TOL n LIMITS +0.2 -0.1     the limits themselves, 80.2 over 79.9
SHEET TOL n FIT H7 [VALUES]      an ISO 286 fit: Ø20 H7, with VALUES Ø20 H7 (+0.021/0)
SHEET TOL n BASIC                a basic (theoretically exact) size, boxed
SHEET TOL n NONE
SHEET DIM ... TOL 0.1            the same, as the dimension is made
```

`n` is the dimension's number in the `SHEET` listing, which also shows each toleranced
size's limits.

**ISO 286 fits:**

| | Letters | Grades | Sizes |
| --- | --- | --- | --- |
| Holes | D, E, F, G, H, JS | IT5 to IT11 | up to 500 mm |
| Shafts | d, e, f, g, h, js, k, m, n, p | IT5 to IT11 | up to 500 mm |

The deviations come from the standard's tables, for example H7 on 20 mm is +0.021/0 and g6
is −0.007/−0.020. The hole letters K, M, N and P need the standard's corrections, so they
are refused rather than guessed.

**General tolerances:** `SHEET TOLERANCE ISO 2768-mK` notes them above the title block
(classes f, m, c and v, with H, K or L for geometric tolerances). They apply to every size
without its own tolerance.

### Feature control frames

```text
SHEET GDT symbol tolerance [DIA] [MMC|LMC] [DATUMS A [B [C]]] ON view x,y,z [OFFSET dx,dy]
SHEET GDT symbol tolerance ... DIM n             under dimension n's value
SHEET GDT DELETE n · SHEET GDT CLEAR
```

A frame reads `| symbol | [Ø] tolerance [Ⓜ|Ⓛ] | A | B | C |`. It sits on a leader to the
model point, `OFFSET` sheet mm away (default 14, 10), or under a dimension's value. Under a
size, it applies to the feature of size, for example a hole's position.

| Kind | Symbols | Datums |
| --- | --- | --- |
| Form | `straightness`, `flatness`, `circularity`, `cylindricity` | none |
| Profile | `profile-line`, `profile-surface` | optional |
| Orientation | `angularity`, `perpendicularity`, `parallelism` | at least one |
| Location | `position`, `concentricity`, `symmetry` | at least one |
| Runout | `runout`, `total-runout` | at least one |

The rules of ISO 1101 / ASME Y14.5 are checked:

- form tolerances refuse datums;
- orientation, location and runout tolerances need one;
- a cylindrical zone (`DIA`) is only for straightness, orientation, position and
  concentricity;
- a material condition (`MMC`, `LMC`) is only for features of size;
- each datum is used once.

A datum reference may carry its own modifier, as `B(M)`.

### Datums and surface finish

```text
SHEET DATUM A ON view x,y,z [OFFSET dx,dy]       the letter boxed, a filled triangle on the feature
SHEET DATUM DELETE A
SHEET FINISH [RA] 1.6 [MACHINED|UNMACHINED] ON view x,y,z [OFFSET dx,dy]
SHEET FINISH DELETE n
```

**Datums:** one letter each, not I, O or Q. A frame naming a datum the drawing doesn't have
yet gets a note.

**Surface finish:** the ISO 1302 mark, with its Ra (µm) over the long leg:

- a bar closes the V when material must be removed (`MACHINED`);
- a circle sits in it when material may not be removed (`UNMACHINED`).

**In the drawing window:** pick *Feature Frame (GD&T)*, *Datum* or *Surface Finish* in the
tool list. Set the symbol, tolerance, Ø, material condition and datums (or the letter, or
Ra and process) in the bar, then click the feature's point in a view. The datum letter steps
on to the next one after each.

**Exports:** PDF and SVG draw the symbols as lines. DXF writes them as lines too, and writes
the ±, Ø and ° signs as its `%%p`, `%%c` and `%%d` codes.

## Export

| Format | Contents |
| --- | --- |
| **PDF** | Vector, the sheet at true size (print at 100% for a scale drawing). Text in Helvetica |
| **SVG** | Vector, mm units, dashed hidden and chain centre lines |
| **DXF** (R12) | Lines and text in mm on layers VISIBLE, HIDDEN, CENTER, THIN (dimensions and hatching), BORDER and CUT, with the HIDDEN and CENTER line types. Opens in AutoCAD, DraftSight, LibreCAD and QCAD |

Exports use a finer hidden-line test than the drawing window.

## For AI agents

The MCP tool `drawing` does all of this. Its operations are:

- `new`, `list` and `delete`;
- `view_add`, `view_move`, `view_delete`, `view_scale` and `view_hidden`;
- `section`, `dim` (with `tolerance`), `dim_delete` and `autodim`;
- `tolerance`, `general_tolerance`, `gdt`, `gdt_delete`, `datum`, `datum_delete` and `finish`;
- `title` and `note`;
- `scale`, `size` and `projection`;
- `export`.

```json
{"operation": "tolerance", "index": 3, "tolerance": "FIT H7 VALUES"}
{"operation": "datum", "letter": "A", "view": "Front", "point": [-40, 0, 6], "sheet_offset": [-12, 0]}
{"operation": "gdt", "symbol": "position", "value": 0.05, "diameter": true, "material": "MMC", "datums": ["A", "B"], "index": 4}
{"operation": "finish", "value": 0.8, "machined": true, "view": "Front", "point": [0, 0, 12]}
```

`run_command` takes any `SHEET` line too.

# 3D printing

SI CDM prepares a part for a 3D printer: it checks the part, repairs the mesh, turns the part so
it needs the least support, and writes a 3MF package or G-code for an FDM printer. Use
*Manufacture → 3D Print*, the command line, or the `print` MCP tool. Each command works on the
selected body unless you name one.

## Check: PRINTCHECK

```text
PRINTCHECK ["body"] [OVERHANG deg] [WALL mm] [BUILD x,y,z]
```

```text
Block 1: ready to print
  48 triangles, 1 shell(s), closed
  size 50 x 10 x 34 mm, fits the build volume, volume 5.8 cm3, area 28.4 cm2
  overhangs past 45 deg: 4.0 cm2 need support (try ORIENT)
  walls: thinnest 6.31 mm, none under 0.8 mm
  about 4 g, 1.3 m of filament, 15 min at 20% infill
```

The check reports:

- **Watertight:**
  - Open edges, which mean holes.
  - Non-manifold edges, where three or more faces share an edge.
  - Inside-out shells, measured by signed volume.
  - The number of separate shells.
  A part is *ready to print* when it is closed, manifold, the right way out and inside the build
  volume.
- **Overhangs:** the area of downward faces that lean past `OVERHANG` degrees from vertical (default
  45°). Faces resting on the bed are not counted.
- **Thin walls:** up to 600 points spread over the surface are each measured by a ray cast inward.
  Points thinner than `WALL` (default 0.8 mm, about two lines from a 0.4 mm nozzle) are counted.
- **Build volume:** the part must fit the `BUILD` size (default 256 × 256 × 256 mm).
- **Estimates:** grams of PLA (1.24 g/cm³), metres of 1.75 mm filament, and minutes. They assume
  walls plus 20% infill.

## Repair: REPAIR

```text
REPAIR ["body"]
```

REPAIR fixes these problems, in this order:

1. Welds seams.
2. Drops degenerate (zero-area) faces and duplicate faces.
3. Makes the winding consistent across each shell.
4. Turns any shell with negative volume the right way out.
5. Removes specks, meaning shells under 1/10 000 of the total area.
6. Fills holes.

The report counts each fix and says whether the result is closed. REPAIR can be undone with UNDO.

## Orient: ORIENT

```text
ORIENT ["body"] [OVERHANG deg]
```

ORIENT tries these directions as "down":

- the six axis directions;
- the normals of the 40 largest faces.

It keeps the one that leaves the least support area. Ties go to the lowest part. The part is then
set on the bed with its lowest point at z = 0.

For example, a T standing on its stem needs 4 cm² of support under its arms. ORIENT lays it down
so that nothing overhangs.

## Slice: SLICE

```text
SLICE ["body"] TO part.gcode [LAYER h] [WIDTH w] [INFILL %] [WALLS n] [TOP n] [BOTTOM n]
      [NOZZLE degC] [BED degC] [SPEED mm/s] [FILAMENT d] [RETRACT mm]
```

| Setting | Default |
| --- | --- |
| Layer height | 0.2 mm |
| Line width | 0.45 mm |
| Infill | 20% |
| Walls | 2 |
| Top / bottom solid layers | 4 / 4 |
| Nozzle / bed | 210 °C / 60 °C |
| Speed (first layer) | 50 mm/s (25) |
| Filament | 1.75 mm |
| Retraction | 0.8 mm |

The layer height can be at most 80% of the line width. A thicker layer can't be squashed onto
the one below it.

The slicer works in these steps:

1. **Layers:** the part is cut at the middle of each layer. The cut segments are chained into closed
   outlines, with the material on the left.
2. **Walls:** each outline is offset inward by half a line, then by one line per further wall,
   using mitred corners.
3. **Infill:** rectilinear lines at ±45°, alternating by layer and clipped even-odd inside the
   innermost wall. The top and bottom layers are solid; the layers between use the infill spacing.
4. **Moves:** the extrusion amount comes from the line's cross-section. The filament retracts on
   travel moves over 2 mm.
5. **Output:** Marlin-style G-code:
   - heat and home (`M140`/`M190`, `M104`/`M109`, `G28`);
   - absolute extrusion (`M82`);
   - the part centred on the bed;
   - `;LAYER:n` and `;TYPE:WALL-OUTER` / `WALL-INNER` / `SOLID-FILL` / `FILL` comments, which G-code
     viewers understand;
   - the fan from layer 2;
   - at the end, the nozzle lifts 10 mm, the heaters turn off and the motors are released.

The report gives the layer count, the filament used and the time. The time counts every move at
its feed rate.

## 3MF: EXPORT

```text
EXPORT part.3mf
```

EXPORT writes the visible bodies as a 3MF package (the 3D Manufacturing Format), in millimetres,
with one named object per body. Fusion, Shapr3D, PrusaSlicer, Bambu Studio, Cura and Orca Slicer
all read 3MF. The package is a standard ZIP (stored entries, CRC-32) holding
`[Content_Types].xml`, `_rels/.rels` and `3D/3dmodel.model`.

STL and OBJ export still work: `EXPORT part.stl`.

## Over MCP

The `print` tool takes these operations:

| `operation` | Parameters |
| --- | --- |
| `check` | `overhang`, `min_wall` |
| `repair` | none |
| `orient` | `overhang` |
| `slice` | `path`, plus optional `layer`, `width`, `infill` (percent), `walls`, `nozzle`, `bed` |
| `export_3mf` | `path` |

Every operation also takes `body`.

```json
{"operation": "slice", "body": "Bracket", "path": "bracket.gcode", "layer": 0.16, "infill": 25}
```

## Example

```text
BOX 10 10 30 @0,0,0
BOX 50 10 6 @0,0,28
BOOLEAN "Block 1" UNION "Block 2"
PRINTCHECK            # 4.0 cm2 needs support
ORIENT                # laid down: no support
SLICE TO t-part.gcode LAYER 0.2 INFILL 20
EXPORT t-part.3mf
```

# Car design, A to Z

The plan for designing a whole car in SI-CDM, from the package to the clay mill, with tools of the
kind automotive studios use for concept and Class-A surfacing. Each step is a feature of its own,
built, tested and documented in turn. This page says what is there now and what comes next.

Status: **done** works and is documented; **planned** is not built yet. The wider plan for
industrial, Class-A and robot product design is the [design roadmap](design-roadmap.md).

## A. Class-A curves and surfaces

| # | Step | Status | Where |
| --- | --- | --- | --- |
| A1 | CV curves: control vertices, degree 1-7, spans, knot insertion, rebuild, fit; ends matched at G0/G1/G2/G3 and measured | **done** | `CVCURVE`, [CAD: CV curves](cad.md#cv-curves-class-a) |
| A2 | Blend curve between two curves (G0-G3 at each end, tension); curves projected onto surfaces and solids (along a direction or to the nearest points, within a tolerance) | **done** | `CVCURVE BLEND`, `CVCURVE PROJECT` |
| A3 | Surface align: a surface's edge matched to another surface or curve at G0-G3 | planned | |
| A4 | Rail surfaces (profiles along one or two rails), square (four sides, G1/G2 to neighbours), skin | planned | |
| A5 | Surface fillet (rolling ball, curvature-continuous option), freeform blend, flange | planned | |
| A6 | Evaluation: highlight lines, curvature combs on curves and sections, edge continuity report, draft map | planned (zebra and curvature maps exist: `ANALYZE`) | |
| A7 | Dynamic shape modelling: lattice, bend, twist, taper on surfaces and curves | planned | |
| A8 | Subdivision cage modelling turned into B-spline patches | planned | |
| A9 | Reverse engineering: surfaces fitted to scans with a deviation map | planned (point clouds exist: `POINTCLOUD`) | |

Already there and used along the way: lofted, boundary (Coons), network (Gordon), extruded,
revolved, offset and blend surfaces, surface fairing, thicken and knit (`SURFACE`, `THICKEN`,
`KNIT`), zebra and curvature analysis.

## B. The car

| # | Step | Status |
| --- | --- | --- |
| B1 | Package: wheelbase, track, overhangs, ground clearance, wheel and tyre sizes, driver and passenger H-points with mannequins, vision angles, approach / departure / breakover angles, a package report | planned |
| B2 | Blueprint underlays: side, front, top and rear images scaled and aligned to the wheelbase | planned (reference images exist: `IMAGE`) |
| B3 | Body surfacing: symmetry plane with continuity across it, character lines, body side, bonnet, roof, glasshouse; a starting body from the package proportions | planned |
| B4 | Wheels and tyres from a size code (e.g. 245/40 R19): tyre, rim with spokes, brake disc and caliper, at the wheel centres | planned |
| B5 | Panel break-up: shut lines cut into the body as gaps with flush and return flanges; the panel list | planned |
| B6 | Lamps, glass and grille | planned |
| B7 | Interior basics: seats, instrument panel and steering wheel from the H-points | planned |
| B8 | Aerodynamics estimate: frontal area, drag coefficient and a pressure map, with its accuracy stated | planned |
| B9 | Visualisation: car paint (metallic flake, clear coat), tyre rubber, chrome, tinted glass; studio and turntable | planned (materials, rendering and turntable video exist: `MATERIAL`, `RENDER`, `VIDEO`) |
| B10 | Hand-off: panels thickened, STEP / IGES out, a package drawing, a 5-axis finishing path for a clay model | planned (STEP, IGES, drawings and 5-axis CAM exist) |
| B11 | A complete sample car, built from the package to the render, in the user guide | planned |

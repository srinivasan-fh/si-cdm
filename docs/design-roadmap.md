# Product roadmap: one product from concept to production

SI-CDM is one program for the whole of a product's development: concept and Class-A styling,
mechanical engineering, simulation, manufacturing, documentation and data management, on one
project and one model. This page says what it has and what it still needs, in three parts:

- **Part I**: concept, industrial and Class-A surface design (the feature areas of studio tools
  such as Autodesk Alias), for cars ([car design, A to Z](car-design.md)) and robots.
- **Part II**: mechanical engineering (the feature areas of tools such as Autodesk Inventor).
- **Part III**: how the stages join into one product, and the work that joins them.

# Part I: Concept, industrial and Class-A design

**Have**: built, tested and documented. **Partly**: some of it is there (what is missing is said).
**Planned**: not built; the code in brackets is its task. Each planned task gets its own tests,
documentation, German text and an MCP tool where it makes sense, as every feature does.

## 1. Concept design and ideation

| Feature | Status |
| --- | --- |
| 2D sketching, 2D curve creation | **Have**: lines, arcs, circles, splines, conics, text... on any plane |
| Reference image placement, design overlays | **Have**: `IMAGE` on sketch planes with opacity |
| Proportion, silhouette and volume studies | Planned (C5): silhouette display, proportion grids, volume and bounding comparisons |
| Design alternatives and iterations | **Partly**: variables, configurations, revisions, `COMPARE`; side-by-side variants planned (C6) |
| Design visualisation, concept presentation | **Partly**: real-time and path-traced rendering, turntable video; presentation boards planned (C4) |
| Freehand drawing | Planned (C1): mouse or pen strokes smoothed into fitted curves |
| Digital sketching | Planned (C2): a raster sketch pad with pressure brushes and layers |
| Image tracing | Planned (C3): outlines of a reference image traced into curves |
| Mood-board references | Planned (C4) |
| Sketch-to-3D, form exploration from 2D references | Planned (C7): side, top and front outlines to a volume |

## 2. Curves

| Feature | Status |
| --- | --- |
| Line, polyline, arc, circle, ellipse, rectangle, polygon | **Have** |
| Freeform, edit-point (interpolated) curves, fitting | **Have**: `SPLINE`, `PATH3D SPLINE`, `CVCURVE FIT` |
| CV curves, degree change, rebuild, matching G0-G3, continuity check | **Have**: `CVCURVE` |
| Bézier curves | **Have** as CV curves with degree + 1 CVs; rational NURBS (weights) planned (D1) |
| Trim, extend, offset, duplicate | **Have** on sketch planes; in 3D planned (D2) |
| Join, split, transform, align curves | Planned (D2) (alignment by `CVCURVE MATCH` **have**) |
| Curvature analysis (combs) | Planned (D3) |
| Projecting curves onto surfaces and solids, blend curves | **Have**: `CVCURVE PROJECT`, `CVCURVE BLEND`; curves kept in a surface's own parameters planned (D4) |
| Intersection curves, isoparametric curves | Planned (D4) |

## 3. Surfaces

| Feature | Status |
| --- | --- |
| NURBS surfaces: loft, skin, multi-section, ruled, boundary, network, extruded, revolved, planar, offset, blend | **Have**: `SURFACE` |
| Thicken, join (knit), transform, duplicate | **Have** |
| Trimmed surfaces | **Partly**: read from STEP/IGES; trimming by curves planned (E2) |
| Rail surfaces, sweep surfaces | Planned (A4, E1) |
| Fillet surfaces | Planned (A5) |
| Surface extension, trimming, splitting, untrimming | Planned (E2) |
| Surface rebuild, merging patches, CV editing of surfaces | Planned (E3) |
| Surface deformation | Planned (A7) |
| Continuity control between surfaces | Planned (A3) |
| Edge and topology management | **Partly**: open edges reported; edge inspection planned (F1) |

## 4. Class-A surfacing

| Feature | Status |
| --- | --- |
| G0-G3 between curves | **Have**: `CVCURVE MATCH`, `CONTINUITY` |
| G0-G3 between surfaces, curve-to-surface matching | Planned (A3) |
| Variable-radius fillets | **Have** on solids; on surfaces planned (A5) |
| Multi-surface blends, corner transitions | **Partly**: two-surface `BLEND`; corners planned (A5) |
| Zebra stripes, curvature maps, fairing | **Have**: `ANALYZE`, `SURFACE FAIR` |
| Combs, reflection and highlight lines, deviation, gap and discontinuity detection, quality validation | Planned (A6, F1) |

## 5. Subdivision modelling

| Feature | Status |
| --- | --- |
| Smooth subdivision, retopology to quads, sculpting | **Have**: `SUBDIVIDE`, `RETOPO`, `SCULPT`, `MULTIRES` |
| Box and edge-loop modelling: vertex, edge, face edits, extrude, bevel, inset, bridge, loop cut, crease | Planned (G1) |
| Subdivision to NURBS | Planned (A8) |
| Mesh to SubD | Planned (G2) |

## 6. Solid and mechanical form

| Feature | Status |
| --- | --- |
| Primitives, booleans, trim, split, transforms, shell, thickness, draft, holes, ribs, grooves, parting, clearance | **Have** |
| Housing and enclosure design | **Partly**: shell, bosses, ribs; an enclosure generator (walls, lid split with a lip, screw bosses, vents) planned (H1) |

## 7. Mesh and reverse engineering

| Feature | Status |
| --- | --- |
| STL, OBJ, 3MF, PLY... import; point clouds; cleanup, repair, reduction, smoothing, remeshing, subdivision, retopology | **Have** |
| Mesh sections | **Have**: `PROJECT` where a plane cuts a body, `SECTION` |
| Surface fitting, surface from mesh, global refitting, deviation | Planned (A9) |
| Scan alignment, reference mesh comparison | Planned (I1) |
| Mesh to SubD | Planned (G2) |

## 8. Parametric and computational design

| Feature | Status |
| --- | --- |
| Variables, equations, configurations, design tables, feature history, scripts (`RUN`), plugins | **Have** |
| Uniform and 2D arrays, circular patterns | **Have**: `PATTERN` |
| Node-based visual programming | Planned (J1) |
| 3D arrays, along a curve, on a surface, non-uniform spacing, instancing | Planned (J2) |
| Parametric patterns on surfaces (perforations, hexagon and Voronoi vents, knurls) | Planned (J3) |
| Reusable templates, custom tools, tool presets | Planned (J4) |

## 9. Industrial product design

Consumer electronics, automotive exterior and interior, lighting, dashboards, seating, casings,
furniture, appliances, wearables, medical and industrial housings are uses of the tools above;
the car-specific ones are the B tasks of the [car plan](car-design.md).

| Feature | Status |
| --- | --- |
| Parting lines, component separation | **Have**: `MOLD`, `SPLIT` |
| Ergonomic forms | **Partly**: humanoid mannequins; car package mannequins planned (B1) |
| Panel gaps | Planned (B5) |
| Seams and stitches, material boundaries | Planned (K1) |

## 10. Robot product design

| Feature | Status |
| --- | --- |
| Shells round a robot's parts with a clearance | **Have**: `ARMOR` |
| Humanoid cover set (head, neck, torso, shoulders, arms, elbows, wrists, hands, hips, thighs, knees, shins, ankles, feet) with split lines, joint clearance and access panels | Planned (L1) |
| Motor, gearbox, sensor, camera, LiDAR, gripper and end-effector housings, cable covers, guards | Planned (L2) |
| Vents, access panels, decorative patterns, logos | **Partly**: `EMBOSS`, `ENGRAVE`; vents and panels planned (J3, L1) |

## 11. Surface analysis and quality

| Feature | Status |
| --- | --- |
| Zebra, curvature, draft angle | **Have**: `ANALYZE`, `MOLD DRAFT` |
| Reflection lines, isophotes, light tunnel, highlight lines, clay shading, normals, edge inspection, gaps, tangency, deviation, diagnostics | Planned (F1) |

## 12. Materials, colours and textures

| Feature | Status |
| --- | --- |
| Metals, plastics, rubber, glass, finishes, roughness, colour and normal maps, UVs, library and swatches | **Have**: `MATERIAL`, `UV` |
| Fabric and leather, procedural textures (carbon fibre, grain), material variants and colour combinations, car paint | Planned (M1, B9) |

## 13. Rendering and visualisation

| Feature | Status |
| --- | --- |
| Real-time shaded display, environment lighting, reflections, path-traced pictures, turntable and animation video, screenshots | **Have** |
| Named cameras, snapshots, side-by-side comparisons, light tunnel | Planned (N1) |
| VR / XR review | Planned (N2): through OpenXR |
| VRED live link | Not planned: proprietary; glTF and FBX carry models with materials |

## 14. Assemblies and structure

| Feature | Status |
| --- | --- |
| Assemblies, external part files, hierarchy, updates, history, design variants | **Have** |
| Layers, selection sets, folders, an object lister with filters | Planned (O1) |

## 15. Communication and documentation

| Feature | Status |
| --- | --- |
| Section views, drawings, design history, comparisons, revisions, vault, author and part data | **Have** |
| 3D annotations, markup, review comments | Planned (O2) |
| Animation editor (key frames for cameras and parts) | **Partly**: motions with key frames; a timeline editor planned (O3) |
| Snapshots, shared views, presentations | Planned (N1, C4) |

## 16. Files and exchange

| Feature | Status |
| --- | --- |
| STEP, IGES, STL, OBJ, 3MF, glTF, FBX, PLY, images, scans, PDF and SVG line work | **Have** |
| JT (ISO 14306), Parasolid text (published formats); layered PSD out for paint programs | Planned (P1) |
| CATIA, SOLIDWORKS and Inventor native files | Not planned: closed formats; they exchange through STEP and IGES |

## 17. Workflow and productivity

| Feature | Status |
| --- | --- |
| Construction history, shortcuts, scripts, plugins, transforms with typed values, snapping, batch operations on selections, data management | **Have** |
| Marking (radial) menus, custom shelves, tool presets, pivot editing, selection filters | Planned (Q1, J4) |
| Automatic symmetry for surfaces | Planned (B3) |

## 18. Newer capabilities

Mesh-to-SubD (G2), sketch-to-3D exploration (C7), surface diagnostics and angle analysis (F1),
deviation tools (A9, I1), VR review (N2) and an aerodynamics estimate (B8) are in the plan above.

# Part II: Mechanical engineering

| Area | Have | Planned |
| --- | --- | --- |
| 1. Sketching and constraints | Lines, polylines, circles, arcs, rectangles, polygons, ellipses, splines, slots, fillets, chamfers, offsets, projected and cut edges, construction geometry, patterns, mirrors, trim, extend; coincident, horizontal, vertical, parallel, perpendicular, tangent, equal, concentric, on, midpoint, fix; driving dimensions, degrees of freedom, variables | Centre lines and centre points, splitting, scaling and rotating sketch geometry, symmetry constraints, driven (reference) dimensions (ME1) |
| 2. Part modelling | Extrude, revolve, sweep, loft, coil (`SPRING`, helical sweep), holes, threads, fillets, chamfers, shell, rib, emboss, split, booleans, thicken, face delete / move / offset, draft, rectangular and circular patterns, mirror, work planes, axes and points, multi-body parts, suppression, reordering, history, parameters and equations | Web, replace face, coordinate systems, derived components (ME2) |
| 3. Advanced modelling | Parametric, direct, surface, solid and hybrid modelling, push-pull, sculpting, boundary patches, stitching, thickening, surface to solid, imported B-rep editing | Healing, simplification and feature recognition of imported geometry (ME3); edge and vertex manipulation, SubD (G1), surface trim and extend (E2) |
| 4. Assemblies | Assemblies and sub-assemblies, linked part files, placement, duplication, mates (coincident, flush, concentric, distance, angle, parallel, perpendicular), fixing, hinge and ball joints, contact in motion studies, configurations, interference and clearance, hierarchy, large-model speed | Rigid, slider, cylindrical, pin-slot and planar joints with limits; insert and tangent mates; replacing, mirroring and patterning components; simplification and shrinkwrap (ME4) |
| 5. Components | Spur, helical and bevel gears, racks, worm drives, planetary sets, belt and chain drives, coil springs, motors, standard bolts, nuts, washers and pins | Shaft generator, bearing library, extension and torsion springs, cams and followers, keys, splines, rivets, couplings, clutches, brakes, press fits (ME5) |
| 6. Engineering calculators | FEA, mass properties, tolerance stack-up | Shafts, bearings (L10 life), gears, belts, chains, springs, bolts, welds, keys and splines, press fits, power / torque / speed, safety factors, sizing and selection (ME6) |
| 7. Sheet metal | Base plates, flanges, hems, mitres, jogs, corner relief, bends from a K factor, flat patterns, DXF out, solid to sheet metal | Contour flange and roll, fold, unfold and refold, corner seams and rounds, punches, rips, bend tables and rules (ME7) |
| 8. Frames | Weldments: SHS, RHS, CHS, flat, round, angle, channel and I profiles along skeleton sketches, mitred and butted ends, cut list | End treatments (notches, lengthening), beam analysis of frames, frame drawings (ME8) |
| 9. Tube, pipe and cable | Pipe, tube, bar, cable and hose routes with bends, bend tables and cut lengths | Fittings, valves, flanges, route constraints; wire harnesses: connectors, branches, wire lengths, flattened harness drawings (ME9) |
| 10. Robot mechanisms | Humanoid and arm libraries, motors, gear trains, robot rigs with joints and limits | Harmonic, planetary and cycloidal drive mounts, finger linkages and tendons, mechanical stops, motor plates, a humanoid's internal frame from its rig (ME10; covers in L1, housings in L2) |
| 11. Motion | Motions, motion studies with plots, contact, collisions, gear and belt links, physics and multibody dynamics | Joint forces and torques, reaction forces, cam-follower motion, results export (ME11) |
| 12. Stress analysis | Linear static stress, natural frequencies, heat, forces, pressures, bearing loads, gravity, fixings, materials, von Mises, displacement, safety factor, reports | Contact between parts, principal stress and strain, mesh refinement, reaction forces, stress-concentration hot spots (ME12) |
| 13. Shape optimisation | Topology optimisation for fixings and loads, volume share; lightweight lattices | Preserve and obstacle regions, manufacturing constraints, smoothed editable results, comparisons (ME13) |
| 14. Drawings | Base, projected, isometric, section and detail views, dimensions (linear, aligned, diameter, radius, angle), auto-dimensions, tolerances and fits, GD&T, datums, surface finish, hole and thread notes, parts lists and balloons, title blocks, PDF / SVG / DXF | Auxiliary, broken, break-out and crop views; baseline and ordinate dimensions; weld and edge symbols; revision tables; templates and standards (ME14). DWG: not planned (closed format; DXF instead) |
| 15. Model-based definition | | 3D dimensions, tolerances, datums and notes on the model; PMI out to STEP AP242 (ME15) |
| 16. BOM | Parts lists, part numbers, descriptions, materials, properties, revisions, configurations | Structured and parts-only BOMs, purchased and made parts, custom columns, CSV / spreadsheet export (ME16) |
| 17. Design automation | Parameters, equations, configurations, design tables (CSV), scripts, history rebuilds | A rules engine: conditional rules, forms, validation, event triggers, spreadsheet-driven design, batch runs, automatic drawings and BOMs (ME17) |
| 18. API | Plugin API, MCP server, Python client for robot environments, command scripts | A documented C++ and Python API over the model with events (ME18) |
| 19. Rendering | Materials, libraries, colours, textures, lights, shadows, reflections, environments, cameras, perspective and orthographic, path tracing, exploded views, animation, video | (Part I, N1) |
| 20. Manufacturing preparation | Drawings, flat patterns, bend data, weldment cut lists, DXF / STEP / STL, materials, parts lists, revisions; and CAM itself (3-5 axis milling, turning, posts), 3D-print slicing | Assembly instructions (with exploded steps) (ME16) |
| 21. Data management | Vault check-in / check-out, versions, revisions, references, comparisons | Web review, redlines and comments (O2) |
| 22. Interoperability | DXF, STEP, IGES, STL, OBJ, 3MF, glTF, FBX | SAT (ME21), Parasolid text and JT (P1). Not planned: DWG, CATIA, SOLIDWORKS, Revit native files (closed; through STEP / IGES) |
| 23. PCB and electronics | | Board outlines, components and holes from IDF / IDX, enclosure fit and clearance (ME19) |
| 24. Libraries and families | Library parts, fasteners, gears, materials, configurations, reference geometry, user properties | A content browser for bearings, springs and custom libraries; part families (model states) (ME20) |
| 25. BIM | | Not planned: outside SI-CDM's purpose |

# Part III: One product

Each stage hands its result to the next inside the same project, without exporting:

| Stage | In SI-CDM | Hands on |
| --- | --- | --- |
| Concept | Sketches, reference images, CV curves, sculpting, armatures | Curves and forms to the Class-A surfaces |
| Class-A styling | Surfaces, continuity, analysis | Surfaces thickened and shelled into parts, panel and cover lines |
| Mechanical design | Parts, assemblies, components, sheet metal, frames, routes | The assembly to simulation, manufacturing and documentation |
| Robotics | Rigs from the assembly, kinematics, dynamics, sensors, learning environments, URDF / SDF / MJCF | Loads and motions back to FEA and motion studies |
| Simulation | FEA, motion studies, physics, topology optimisation | Changes to the design (via parameters and the history) |
| Manufacturing | CAM, turning, 3D printing, sheet metal flats, cut lists | G-code, DXF, STL, print files |
| Documentation | Drawings, GD&T, parts lists, renders, videos | The release |
| Data | Vault, revisions, comparisons | The released revision locks the design for manufacturing |

What joins them still needs work:

| Task | What |
| --- | --- |
| U1 | Workspaces over one project (Design, Engineer, Robot, Simulate, Make, Document, Manage), each with its own tools and panels, and a guide that walks a product through them |
| U2 | Associativity across stages: parts made from Class-A surfaces update when the surfaces change; drawings, BOMs, CAM, FEA and robot rigs are marked out of date (and rebuilt on request) when the model they came from changes |
| U3 | One product structure: assemblies, parts, surfaces, frames, routes, harnesses and boards in one tree that feeds the BOM, drawings, rig and exports |
| U4 | Complete sample products, each run end to end in CI: a robot arm (concept, covers, mechanism, FEA, CAM, drawings, BOM), a humanoid leg, and the car (B11) |
| U5 | One automation surface: every stage reachable by commands, MCP, the API and rules alike, so a script or an AI assistant can take a product through the whole pipeline |

## Where SI-CDM sits in a robot company's tool chain

SI-CDM already covers more than styling: mechanical CAD (sketches, features, assemblies,
drawings, tolerances), linear FEA (stress, modes, heat), CAM (3-5 axis milling, turning), 3D
printing, robot rigs with kinematics, dynamics and sensors, and an MCP interface. These are
good for design work and early checks; for certified analysis, large-scale simulation and
production they do not replace dedicated tools (ANSYS-class FEA, Isaac Sim / MuJoCo-class robot
simulation at scale, NX / CATIA for very large assemblies). PCB and IC design, firmware and
embedded software are outside SI-CDM altogether.

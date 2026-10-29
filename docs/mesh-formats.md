# Mesh formats: 3MF, AMF, PLY, OFF, glTF and VRML

Besides STEP and IGES (exact solids, see [STEP and IGES](step.md)) and STL and OBJ, SI-CDM
imports the mesh formats that printers, scanners, viewers and other CAD tools exchange. Each
part comes in as a body with its name, its place in the assembly tree, its colour, and, where
the file has them, its material and the colours of its faces.

```text
IMPORT bracket.3mf                 (or File → Import...)
IMPORT scan.ply UNIT inch          the file doesn't say its unit: one unit is an inch
IMPORT pcb.wrl UNIT mm             VRML written in millimetres (many CAD tools do)
IMPORT robot.glb
```

The readers are SI-CDM's own code, zip and deflate included. No outside library is used.

| Format | Extensions | What is read |
| --- | --- | --- |
| 3MF | `.3mf` | objects, components (nested, from other model parts too), build items and their transforms, the unit, base materials and colour groups (a colour per triangle corner). Support objects are left out |
| AMF | `.amf` (plain or zipped) | objects and their volumes, materials and colours, constellations of instances (moved and turned), the unit |
| PLY | `.ply` | ASCII and binary, either byte order: points, polygons (fanned to triangles), vertex colours |
| OFF | `.off` | OFF and COFF: points, polygons, vertex and face colours |
| glTF 2.0 | `.gltf` `.glb` | the scene's node tree and its transforms (matrices or translation, rotation and scale), triangles, strips and fans, indices of any size, sparse accessors, base colour, metallic and roughness, vertex colours. Buffers may be in the `.glb`, embedded or in files beside the `.gltf` |
| VRML 97 | `.wrl` `.wrz` (gzipped too) | Transform, Group, Switch and LOD nodes, Shape with IndexedFaceSet, Material diffuse colour and shininess, DEF and USE |

**Units and axes:**

- 3MF and AMF say their unit, and it is used.
- glTF is in metres with +Y up. It is turned to +Z up and millimetres.
- VRML is in metres by its standard. Many CAD and electronics tools write millimetres or tenths
  of an inch instead, so give `UNIT` when the size comes out wrong.
- PLY and OFF don't say their unit: millimetres are taken unless `UNIT` says otherwise.
- `UNIT` (mm, cm, m, inch, foot, micron) overrides the unit of any of these files.

**What becomes of a part:**

- Each part is a body. A part in a sub-assembly (a 3MF component, a glTF node under a named
  parent, a named VRML group) gets that assembly's path in the Model panel.
- A part placed mirrored keeps its faces pointing out.
- Normals stay smooth across edges shallower than 30° and sharp across steeper ones. A box
  looks like a box and a cylinder looks round.
- A part with one colour everywhere gets it as its own colour. Faces of different colours
  keep theirs as vertex colours, which can be painted over.
- glTF's metallic and roughness become the body's material. VRML's shininess becomes its
  roughness.

**What is left out, and said:**

- Points and lines (glTF), and support objects (3MF).
- VRML Box, Sphere, Cylinder and Cone nodes.
- Compressed glTF meshes (Draco, meshopt): save without compression.
- VRML 1.0, glTF 1.0 and textures.
- A file that places parts inside themselves (a 3MF object among its own components, a glTF
  node its own child, an AMF constellation inside itself) is read without the loop.
- A file whose shared parts multiply past 20,000 parts or 50 million triangles stops there,
  with a warning.

A damaged file is refused with the reason: a checksum that doesn't match, a package without a
model, XML that isn't well formed, or an index outside its list.

Over MCP, `import_file` takes the same files, with `unit` for the unit.

## Native CAD formats

The native files of other CAD systems are not read. These are Parasolid (`.x_t`), ACIS
(`.sat`), JT, CATIA, NX, Creo, SolidWorks and Inventor. Each needs the vendor's own kernel or a
licensed toolkit. Export STEP (AP203, AP214 or AP242) or IGES from those systems instead: the
exact geometry, assembly structure and units come through. See [STEP and IGES](step.md).

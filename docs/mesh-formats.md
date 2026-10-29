# Mesh formats: 3MF, AMF, PLY, OFF, glTF, VRML and FBX

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

## Exporting glTF, GLB and FBX

```
EXPORT robot.glb          glTF 2.0, binary: one file with everything in it
EXPORT robot.gltf         glTF 2.0 as JSON, its data inside as base64
EXPORT robot.fbx          FBX 7.4, binary
```

These are for Blender, game engines (Unity, Unreal, Godot), the web (three.js, Babylon.js,
`<model-viewer>`) and animation tools. The visible bodies are written, each with its name and in
its assembly group.

**What goes into glTF:**

- Positions and normals. The scene's millimetres with +Z up become metres with +Y up, as glTF
  has it.
- Texture coordinates from `UV UNWRAP`. Vertices are split where charts meet, and v is turned to
  glTF's (it counts down).
- Polypaint, as vertex colours.
- Materials as PBR metallic-roughness: the colour, `METALLIC` and `ROUGHNESS`.
  - Glass uses `KHR_materials_transmission` and `KHR_materials_ior`.
  - A glow uses `emissiveFactor`, with `KHR_materials_emissive_strength` above 1.
  - Bodies that look the same share one material.
- The `COLORMAP` and `NORMALMAP` images, embedded. PNG and JPEG go as they are. Other image
  formats are written as PNG.
- Tangents for a normal-mapped body, in the frame `BAKE` bakes the map in, so it shades the same
  in any viewer.
- Armature skins (`ARMATURE ... SKIN`) as glTF skins, which animation tools can pose:
  - Each ball is a joint, under its parent ball and turned as the armature is posed.
  - Each vertex follows up to four joints.
  - The mesh is in its rest pose, so the joints pose it back to the shape seen in the model.
  - Edits sculpted on the skin since its last pose are kept.
  - The armature itself is not written as a mesh.

**What goes into FBX:**

- The same bodies and groups, in centimetres with +Y up.
- Normals, UVs and vertex colours, per polygon corner.
- Materials and the maps, embedded.
- Quad meshes from `RETOPO` stay quads.
- A skin goes as it is posed, without its bones. Use glTF to take the skin and bones.

**What isn't written:**

- Hidden bodies, sketches and reference images.
- Lines and points.
- Animation (keyframed motions).
- A skin that was remeshed since it was skinned. Dynamic topology counts, because it changes the
  vertices. Such a skin goes as it is, and the export says so.

**Checked:**

- The Khronos glTF Validator finds no errors, warnings or hints in the files written.
- Assimp reads both formats back with the same positions, UVs, quads, materials and embedded maps.
- The tests read the files back and pose the skin from its joints. It matches the model's posed
  mesh to within a thousandth of a millimetre.

The FBX has not been opened in Autodesk's own software. Over MCP, `export_file` writes all three.

## Native CAD formats

The native files of other CAD systems are not read. These are Parasolid (`.x_t`), ACIS
(`.sat`), JT, CATIA, NX, Creo, SolidWorks and Inventor. Each needs the vendor's own kernel or a
licensed toolkit. Export STEP (AP203, AP214 or AP242) or IGES from those systems instead: the
exact geometry, assembly structure and units come through. See [STEP and IGES](step.md).

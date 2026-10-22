# Reference IGES files

Written by OpenCascade (used only to make these files, not by SI-CDM) from the shapes in
`../step`, to test the IGES reader against an independent writer. The exact volumes are
the STEP ones (`../step/volumes.inc`).

- `*_faces.igs`: trimmed surfaces (144), the usual IGES exchange form, with no topology
  between faces. The reader sews them into closed shells.
- `*_brep.igs`: B-rep solids (186) with shells, faces, loops, and edge and vertex lists.
- `box_inch_*`: written in inches (the global section's unit), a 40 x 30 x 20 mm block.
- `assembly_*`: the three placed parts of `../step/assembly.step`.

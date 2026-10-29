#pragma once

// Importing the mesh exchange formats: each part a body, with its name, where it sits in
// the assembly, its colour and, where the file has them, its material and vertex colours.
//
//   3MF     the 3D Manufacturing Format package (zip): objects, components, build items and
//           their transforms, units, base materials and colour groups, other model parts
//   AMF     Additive Manufacturing Format (plain or zipped XML): objects, volumes, materials,
//           colours, constellations of instances, units
//   PLY     Stanford polygon files, ASCII and binary (either byte order), vertex colours
//   OFF     Object File Format (and COFF with vertex colours)
//   glTF    glTF 2.0 (.gltf with embedded or separate buffers, .glb): the node tree and its
//           transforms, triangles, strips and fans, materials, vertex colours; metres and
//           +Y up turned to millimetres and +Z up
//   VRML    VRML 97 (.wrl): Transform and Group nodes, Shape with IndexedFaceSet, Material
//           diffuse colours, DEF and USE

#include "core/Math.hpp"
#include "core/Scene.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace sicdm {

struct ImportedPart {
    std::string name;
    std::string assembly; // "Gearbox/Input shaft" ("" the top level)
    std::vector<Vec3> points; // millimetres, +Z up
    std::vector<std::array<std::uint32_t, 3>> triangles;
    std::vector<std::uint32_t> cornerColors; // packed sRGB per corner (three a triangle; 0: none), or empty
    std::optional<Vec3> color;               // the part's colour (sRGB, 0..1)
    std::optional<float> metallic, roughness;
};

struct MeshImportReport {
    std::string format;        // "3MF", "glTF 2.0 (binary)"...
    double unitToMm = 1;       // one file unit in millimetres
    bool unitAssumed = false;  // the file doesn't say (PLY, OFF): millimetres taken
    std::vector<std::string> warnings;
};

// The extensions read here (lower case, with the dot).
bool isMeshModelExtension(const std::string& ext);

// The parts in a mesh model file. Empty (with why) when it can't be read or has none.
// `unitToMm`: one file unit in millimetres, in place of what the file says (or, for PLY and
// OFF, which don't say, of the millimetres taken).
std::vector<ImportedPart> importMeshModel(const std::string& path, MeshImportReport* report = nullptr, std::string* why = nullptr,
                                          std::optional<double> unitToMm = std::nullopt);

// A body from a part: shared corners kept, normals smooth across edges sharper than
// `creaseDeg` only.
Body bodyFromPart(const ImportedPart& part, float creaseDeg = 30.0f);

} // namespace sicdm

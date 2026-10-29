#pragma once

// Exporting the scene for games, the web, renderers and animation tools:
//
//   glTF 2.0   .gltf (one file, its data inside as base64) or .glb (binary): a node per
//              body, grouped by assembly; positions, normals, texture coordinates (UV
//              UNWRAP), polypaint as vertex colours; PBR materials (colour, metallic,
//              roughness, glass by KHR_materials_transmission and _ior, glow by
//              emissive strength) with their colour and normal maps embedded; and the
//              skins made from armatures as glTF skins (a joint per ball, four weights a
//              vertex), posed as the armature is.
//   FBX        binary FBX 7.4: the same bodies, assembly groups, normals, UVs, vertex
//              colours, materials and embedded maps; quad meshes stay quads. Skins go as
//              the posed shape.
//
// The scene is in millimetres with +Z up; glTF is metres with +Y up, FBX centimetres with +Y
// up (as most tools read it). Visible bodies only; armatures go as skeletons, not meshes.

#include "core/Scene.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace sicdm {

struct ModelExportReport {
    std::string format;      // "glTF 2.0 (binary)", "FBX 7.4 (binary)"
    std::size_t bodies = 0, triangles = 0, materials = 0, textures = 0, skins = 0;
    std::vector<std::string> notes; // what was left out and why
};

// .gltf, .glb and .fbx (lower case, with the dot).
bool isModelExportExtension(const std::string& ext);

// Writes the visible bodies to `path` (.gltf, .glb or .fbx by its extension). False (why)
// when there is nothing to write or the file can't be written.
bool exportModel(const Scene& scene, const std::string& path, ModelExportReport* report = nullptr, std::string* why = nullptr);

// The report as one line for the command line.
std::string modelExportSummary(const ModelExportReport& r, const std::string& path);

} // namespace sicdm

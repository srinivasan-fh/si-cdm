#pragma once

// How a body's surface looks in a rendered image (RENDER): a physically based material.
// The colour is the body's own (Body::color, or its polypaint, or a colour map).

#include "core/Math.hpp"

#include <string>
#include <vector>

namespace sicdm {

struct MaterialProps {
    std::string preset;      // the preset it came from ("" plastic: the default)
    float metallic = 0.0f;   // 0 a dielectric (plastic, paint, wood), 1 a metal (its colour tints reflections)
    float roughness = 0.45f; // 0 a mirror, 1 fully matte
    float transmission = 0;  // 1: clear like glass (refracts)
    float ior = 1.5f;        // index of refraction (glass 1.5, water 1.33)
    float emission = 0;      // glows with its colour times this (a lamp, a screen)
    std::string colorMap;    // images on the body's texture coordinates (UV UNWRAP, BAKE)
    std::string normalMap;   // tangent-space normal map
    bool isDefault() const noexcept {
        return preset.empty() && metallic == 0.0f && roughness == 0.45f && transmission == 0 && ior == 1.5f && emission == 0 && colorMap.empty() &&
               normalMap.empty();
    }
};

struct MaterialPreset {
    const char* name;
    const char* description;
    bool setsColor; // metals and glass have their own colour
    Vec3 color;
    float metallic, roughness, transmission, ior, emission;
};

const std::vector<MaterialPreset>& materialPresets();
// Applies a preset by name (case-insensitive); its colour too when it has one. False when
// there is no such preset.
bool applyMaterialPreset(const std::string& name, MaterialProps& material, Vec3* color);

} // namespace sicdm

#include "core/Material.hpp"

#include "core/Translate.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace sicdm {

namespace {

// A body's colour is sRGB; the renderer turns it into linear light with a 2.2 power. A
// metal's colour is its measured reflectance at normal incidence (F0, linear), so it is
// stored as the sRGB that comes back to exactly that.
Vec3 srgbOf(float r, float g, float b) {
    auto e = [](float v) { return std::pow(std::clamp(v, 0.0f, 1.0f), 1.0f / 2.2f); };
    return {e(r), e(g), e(b)};
}
Vec3 hex(unsigned rgb) { return {static_cast<float>((rgb >> 16) & 255) / 255.0f, static_cast<float>((rgb >> 8) & 255) / 255.0f, static_cast<float>(rgb & 255) / 255.0f}; }

// Families, in the order the picker shows them.
constexpr const char* kPlastic = SICDM_TR_NOOP("Plastics and paint");
constexpr const char* kMetal = SICDM_TR_NOOP("Metals");
constexpr const char* kPrecious = SICDM_TR_NOOP("Precious metals");
constexpr const char* kAlloy = SICDM_TR_NOOP("Alloys and finishes");
constexpr const char* kColoured = SICDM_TR_NOOP("Coloured metals");
constexpr const char* kClear = SICDM_TR_NOOP("Glass and liquids");
constexpr const char* kOther = SICDM_TR_NOOP("Other");

MaterialPreset dielectric(const char* name, const char* desc, const char* family, bool sets, Vec3 c, float rough) {
    return {name, desc, family, sets, c, 0, rough, 0, 1.5f, 0};
}
// A metal by its measured reflectance (linear F0).
MaterialPreset metal(const char* name, const char* desc, const char* family, float r, float g, float b, float rough) {
    return {name, desc, family, true, srgbOf(r, g, b), 1, rough, 0, 1.5f, 0};
}
// A coloured metal finish by how it looks (sRGB).
MaterialPreset tinted(const char* name, const char* desc, unsigned rgb, float rough, float metallic = 1) {
    return {name, desc, kColoured, true, hex(rgb), metallic, rough, 0, 1.5f, 0};
}

} // namespace

const std::vector<MaterialPreset>& materialPresets() {
    static const std::vector<MaterialPreset> k = {
        // Plastics and paint: the body's own colour.
        dielectric(SICDM_TR_NOOP("plastic"), SICDM_TR_NOOP("Glossy plastic in the body's colour"), kPlastic, false, {}, 0.35f),
        dielectric(SICDM_TR_NOOP("matte"), SICDM_TR_NOOP("Matte plastic or paint in the body's colour"), kPlastic, false, {}, 0.8f),
        dielectric(SICDM_TR_NOOP("paint"), SICDM_TR_NOOP("Car paint: a deep gloss in the body's colour"), kPlastic, false, {}, 0.12f),
        {SICDM_TR_NOOP("metallic-paint"), SICDM_TR_NOOP("Metallic car paint (flakes under the gloss) in the body's colour"), kPlastic, false, {}, 0.6f, 0.22f, 0, 1.5f, 0},
        dielectric(SICDM_TR_NOOP("rubber"), SICDM_TR_NOOP("Black rubber"), kPlastic, true, {0.05f, 0.05f, 0.05f}, 0.9f),

        // Pure metals, by their measured reflectance.
        metal(SICDM_TR_NOOP("steel"), SICDM_TR_NOOP("Machined carbon steel (iron)"), kMetal, 0.562f, 0.565f, 0.578f, 0.3f),
        metal(SICDM_TR_NOOP("iron"), SICDM_TR_NOOP("Iron"), kMetal, 0.562f, 0.565f, 0.578f, 0.35f),
        metal(SICDM_TR_NOOP("cast-iron"), SICDM_TR_NOOP("Cast iron, as cast: dark and rough"), kMetal, 0.42f, 0.42f, 0.43f, 0.7f),
        metal(SICDM_TR_NOOP("aluminium"), SICDM_TR_NOOP("Machined aluminium"), kMetal, 0.913f, 0.922f, 0.924f, 0.32f),
        metal(SICDM_TR_NOOP("chrome"), SICDM_TR_NOOP("Polished chrome plating"), kMetal, 0.550f, 0.556f, 0.554f, 0.03f),
        metal(SICDM_TR_NOOP("nickel"), SICDM_TR_NOOP("Nickel plating"), kMetal, 0.660f, 0.609f, 0.526f, 0.15f),
        metal(SICDM_TR_NOOP("copper"), SICDM_TR_NOOP("Copper"), kMetal, 0.955f, 0.638f, 0.538f, 0.25f),
        metal(SICDM_TR_NOOP("titanium"), SICDM_TR_NOOP("Titanium"), kMetal, 0.542f, 0.497f, 0.449f, 0.35f),
        metal(SICDM_TR_NOOP("zinc"), SICDM_TR_NOOP("Zinc (galvanised)"), kMetal, 0.664f, 0.824f, 0.850f, 0.4f),
        metal(SICDM_TR_NOOP("tin"), SICDM_TR_NOOP("Tin"), kMetal, 0.79f, 0.79f, 0.80f, 0.3f),
        metal(SICDM_TR_NOOP("lead"), SICDM_TR_NOOP("Lead"), kMetal, 0.632f, 0.626f, 0.641f, 0.55f),
        metal(SICDM_TR_NOOP("cobalt"), SICDM_TR_NOOP("Cobalt"), kMetal, 0.662f, 0.655f, 0.634f, 0.25f),
        metal(SICDM_TR_NOOP("tungsten"), SICDM_TR_NOOP("Tungsten"), kMetal, 0.504f, 0.498f, 0.478f, 0.3f),
        metal(SICDM_TR_NOOP("magnesium"), SICDM_TR_NOOP("Magnesium"), kMetal, 0.85f, 0.85f, 0.86f, 0.35f),
        metal(SICDM_TR_NOOP("mercury"), SICDM_TR_NOOP("Mercury (liquid)"), kMetal, 0.781f, 0.779f, 0.779f, 0.0f),

        // Precious metals.
        metal(SICDM_TR_NOOP("gold"), SICDM_TR_NOOP("Polished yellow gold"), kPrecious, 1.000f, 0.766f, 0.336f, 0.12f),
        metal(SICDM_TR_NOOP("rose-gold"), SICDM_TR_NOOP("Rose gold"), kPrecious, 0.98f, 0.70f, 0.58f, 0.15f),
        metal(SICDM_TR_NOOP("white-gold"), SICDM_TR_NOOP("White gold (rhodium plated)"), kPrecious, 0.86f, 0.84f, 0.80f, 0.1f),
        metal(SICDM_TR_NOOP("silver"), SICDM_TR_NOOP("Polished silver"), kPrecious, 0.972f, 0.960f, 0.915f, 0.08f),
        metal(SICDM_TR_NOOP("platinum"), SICDM_TR_NOOP("Platinum"), kPrecious, 0.673f, 0.637f, 0.585f, 0.1f),
        metal(SICDM_TR_NOOP("palladium"), SICDM_TR_NOOP("Palladium"), kPrecious, 0.733f, 0.697f, 0.652f, 0.12f),
        metal(SICDM_TR_NOOP("rhodium"), SICDM_TR_NOOP("Rhodium plating"), kPrecious, 0.79f, 0.79f, 0.78f, 0.05f),

        // Alloys and finishes.
        metal(SICDM_TR_NOOP("stainless"), SICDM_TR_NOOP("Stainless steel, satin"), kAlloy, 0.66f, 0.64f, 0.61f, 0.25f),
        metal(SICDM_TR_NOOP("brushed"), SICDM_TR_NOOP("Brushed stainless steel"), kAlloy, 0.66f, 0.64f, 0.61f, 0.45f),
        metal(SICDM_TR_NOOP("polished-steel"), SICDM_TR_NOOP("Mirror-polished stainless steel"), kAlloy, 0.66f, 0.64f, 0.61f, 0.04f),
        metal(SICDM_TR_NOOP("brushed-aluminium"), SICDM_TR_NOOP("Brushed aluminium"), kAlloy, 0.913f, 0.922f, 0.924f, 0.5f),
        metal(SICDM_TR_NOOP("brass"), SICDM_TR_NOOP("Brass"), kAlloy, 0.910f, 0.778f, 0.423f, 0.25f),
        metal(SICDM_TR_NOOP("bronze"), SICDM_TR_NOOP("Bronze"), kAlloy, 0.80f, 0.53f, 0.33f, 0.3f),
        metal(SICDM_TR_NOOP("pewter"), SICDM_TR_NOOP("Pewter"), kAlloy, 0.60f, 0.60f, 0.59f, 0.4f),
        metal(SICDM_TR_NOOP("gunmetal"), SICDM_TR_NOOP("Gunmetal: dark blue-grey"), kAlloy, 0.22f, 0.23f, 0.26f, 0.3f),
        metal(SICDM_TR_NOOP("black-chrome"), SICDM_TR_NOOP("Black chrome: a dark mirror"), kAlloy, 0.12f, 0.12f, 0.13f, 0.06f),
        metal(SICDM_TR_NOOP("champagne"), SICDM_TR_NOOP("Champagne: pale warm gold"), kAlloy, 0.85f, 0.72f, 0.50f, 0.2f),
        metal(SICDM_TR_NOOP("patina"), SICDM_TR_NOOP("Weathered copper patina (verdigris)"), kAlloy, 0.30f, 0.58f, 0.48f, 0.75f),

        // Coloured metals: anodised aluminium, heat-tinted titanium, and the body's own colour.
        {SICDM_TR_NOOP("anodised"), SICDM_TR_NOOP("Anodised aluminium in the body's colour"), kColoured, false, {}, 1, 0.35f, 0, 1.5f, 0},
        tinted(SICDM_TR_NOOP("anodised-red"), SICDM_TR_NOOP("Red anodised aluminium"), 0xC4232F, 0.3f),
        tinted(SICDM_TR_NOOP("anodised-blue"), SICDM_TR_NOOP("Blue anodised aluminium"), 0x1E5BD6, 0.3f),
        tinted(SICDM_TR_NOOP("anodised-green"), SICDM_TR_NOOP("Green anodised aluminium"), 0x22A050, 0.3f),
        tinted(SICDM_TR_NOOP("anodised-purple"), SICDM_TR_NOOP("Purple anodised aluminium"), 0x7A2DB8, 0.3f),
        tinted(SICDM_TR_NOOP("anodised-orange"), SICDM_TR_NOOP("Orange anodised aluminium"), 0xE8701C, 0.3f),
        tinted(SICDM_TR_NOOP("anodised-gold"), SICDM_TR_NOOP("Gold anodised aluminium"), 0xD9A93A, 0.3f),
        tinted(SICDM_TR_NOOP("anodised-black"), SICDM_TR_NOOP("Black anodised aluminium"), 0x26272B, 0.35f),
        tinted(SICDM_TR_NOOP("anodised-pink"), SICDM_TR_NOOP("Pink anodised aluminium"), 0xE57AA8, 0.3f),
        tinted(SICDM_TR_NOOP("anodised-teal"), SICDM_TR_NOOP("Teal anodised aluminium"), 0x1F9AA0, 0.3f),
        tinted(SICDM_TR_NOOP("titanium-blue"), SICDM_TR_NOOP("Heat-tinted titanium, blue"), 0x3A6FD8, 0.2f),
        tinted(SICDM_TR_NOOP("titanium-purple"), SICDM_TR_NOOP("Heat-tinted titanium, purple"), 0x7B4FC4, 0.2f),
        tinted(SICDM_TR_NOOP("titanium-gold"), SICDM_TR_NOOP("Heat-tinted titanium, straw gold"), 0xC9A24A, 0.2f),

        // Glass and liquids.
        {SICDM_TR_NOOP("glass"), SICDM_TR_NOOP("Clear glass"), kClear, true, {0.97f, 0.98f, 0.98f}, 0, 0.0f, 1, 1.5f, 0},
        {SICDM_TR_NOOP("tinted"), SICDM_TR_NOOP("Tinted glass in the body's colour"), kClear, false, {}, 0, 0.0f, 1, 1.5f, 0},
        {SICDM_TR_NOOP("frosted"), SICDM_TR_NOOP("Frosted glass"), kClear, true, {0.95f, 0.96f, 0.97f}, 0, 0.35f, 1, 1.5f, 0},
        {SICDM_TR_NOOP("water"), SICDM_TR_NOOP("Water"), kClear, true, {0.9f, 0.96f, 1.0f}, 0, 0.0f, 1, 1.33f, 0},

        // Other.
        dielectric(SICDM_TR_NOOP("ceramic"), SICDM_TR_NOOP("White glazed ceramic"), kOther, true, {0.9f, 0.9f, 0.88f}, 0.1f),
        dielectric(SICDM_TR_NOOP("wood"), SICDM_TR_NOOP("Oiled wood"), kOther, true, {0.55f, 0.36f, 0.2f}, 0.6f),
        {SICDM_TR_NOOP("light"), SICDM_TR_NOOP("A glowing surface in the body's colour (a lamp, a screen)"), kOther, false, {}, 0, 0.5f, 0, 1.5f, 6},
    };
    return k;
}

const std::vector<const char*>& materialFamilies() {
    static const std::vector<const char*> k = {kPlastic, kMetal, kPrecious, kAlloy, kColoured, kClear, kOther};
    return k;
}

const std::vector<MaterialFinish>& materialFinishes() {
    static const std::vector<MaterialFinish> k = {
        {SICDM_TR_NOOP("polished"), 0.04f}, {SICDM_TR_NOOP("satin"), 0.25f}, {SICDM_TR_NOOP("brushed"), 0.45f}, {SICDM_TR_NOOP("matte"), 0.75f}};
    return k;
}

bool applyMaterialPreset(const std::string& name, MaterialProps& m, Vec3* color) {
    std::string n;
    for (char c : name) n += c == '_' || c == ' ' ? '-' : static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    // American spellings and common names.
    static const std::pair<const char*, const char*> kAlias[] = {
        {"aluminum", "aluminium"}, {"anodized", "anodised"}, {"brushed-aluminum", "brushed-aluminium"}, {"stainless-steel", "stainless"},
        {"chromium", "chrome"}, {"verdigris", "patina"}, {"yellow-gold", "gold"}, {"pink-gold", "rose-gold"}};
    for (const auto& [from, to] : kAlias)
        if (n == from) n = to;
    if (n.rfind("anodized-", 0) == 0) n = "anodised-" + n.substr(9);
    for (const MaterialPreset& p : materialPresets()) {
        if (n != p.name) continue;
        m.preset = p.name;
        m.metallic = p.metallic, m.roughness = p.roughness, m.transmission = p.transmission, m.ior = p.ior, m.emission = p.emission;
        if (p.setsColor && color) *color = p.color;
        return true;
    }
    return false;
}

} // namespace sicdm

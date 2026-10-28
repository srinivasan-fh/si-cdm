#include "TestFramework.hpp"
#include "core/Command.hpp"
#include "core/Document.hpp"
#include "core/FileIO.hpp"
#include "core/Render.hpp"
#include "core/Scene.hpp"

#include <filesystem>
#include <string>

using namespace sicdm;

#define REQUIRE(c)                  \
    do {                            \
        const bool required_ = (c); \
        CHECK(required_);           \
        if (!required_) return;     \
    } while (0)

namespace {

CommandOutput run(Document& d, const std::string& line) { return executeCommand(d, line, nullptr); }

float brightness(const std::vector<std::uint8_t>& px, int W, int x, int y, int r = 2) {
    float s = 0;
    int n = 0;
    for (int dy = -r; dy <= r; ++dy)
        for (int dx = -r; dx <= r; ++dx) {
            const std::size_t k = (static_cast<std::size_t>(y + dy) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x + dx)) * 3;
            s += static_cast<float>(px[k]) + px[k + 1] + px[k + 2];
            ++n;
        }
    return s / static_cast<float>(3 * n);
}

} // namespace

TEST_CASE("materials: presets, settings, saved with the project and undone") {
    Document d;
    REQUIRE(run(d, "BOX 20 20 20 @0,0,0").ok);
    CHECK(run(d, "MATERIAL LIST").lines.size() >= 15);
    REQUIRE(run(d, "MATERIAL \"Block 1\" chrome").ok);
    const Body& b = d.scene.bodies()[0];
    CHECK(b.material.preset == "chrome");
    CHECK(b.material.metallic == 1.0f);
    CHECK(b.material.roughness < 0.1f);
    CHECK(b.color.x > 0.7f && b.color.x < 0.8f); // chrome has its own colour: chromium's measured reflectance
    REQUIRE(run(d, "MATERIAL \"Block 1\" ROUGHNESS 0.3 COLORMAP wood.png").ok);
    CHECK(d.scene.bodies()[0].material.roughness == 0.3f);
    CHECK(d.scene.bodies()[0].material.colorMap == "wood.png");
    CHECK(!run(d, "MATERIAL \"Block 1\" unobtainium").ok);
    CHECK(!run(d, "MATERIAL \"Block 1\" ROUGHNESS 3").ok);
    CHECK(run(d, "MATERIAL \"Block 1\"").ok); // what it has
    // Saved and read back.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_material.sicdm").string();
    REQUIRE(run(d, "RENDER ENV sky SUN 200 30 SIZE 320 200 SAMPLES 16").ok);
    std::string err;
    REQUIRE(saveProject(d.scene, d.state, path, &err));
    Scene scene;
    ProjectState state;
    REQUIRE(loadProject(scene, state, path, &err));
    CHECK(scene.bodies()[0].material.preset == "chrome");
    CHECK(scene.bodies()[0].material.roughness == 0.3f);
    CHECK(scene.bodies()[0].material.colorMap == "wood.png");
    CHECK(state.render.environment == "sky");
    CHECK(state.render.sunAzimuth == 200.0f);
    CHECK(state.render.width == 320);
    CHECK(state.render.samples == 16);
    std::filesystem::remove(path);
    // Undone like any other edit.
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.bodies()[0].material.roughness < 0.1f);
    REQUIRE(run(d, "UNDO").ok);
    CHECK(d.scene.bodies()[0].material.isDefault());
}

TEST_CASE("render: a picture with soft shadows on the floor, the same every time") {
    Document d;
    REQUIRE(run(d, "BOX 20 20 80 @0,0,0").ok); // a tall block on the floor
    RenderSettings rs;
    rs.environment = "sky";
    rs.sunAzimuth = 0, rs.sunElevation = 60; // the sun towards +X: the shadow falls towards -X
    rs.width = rs.height = 100;
    rs.samples = 16;
    // Straight down, orthographic, 200 mm across.
    RenderCamera cam;
    cam.position = {0, 0, 500}, cam.forward = {0, 0, -1}, cam.right = {1, 0, 0}, cam.up = {0, 1, 0};
    cam.ortho = true, cam.orthoHeight = 200;
    std::vector<std::uint8_t> a, b;
    RenderStats stats;
    std::string why;
    REQUIRE(renderImage(d.scene, rs, cam, a, &stats, &why));
    REQUIRE(a.size() == 100u * 100u * 3u);
    CHECK(stats.triangles == 12);
    auto pixel = [](float wx, float wy) { return std::pair<int, int>{static_cast<int>((wx / 200.0f + 0.5f) * 100), static_cast<int>((0.5f - wy / 200.0f) * 100)}; };
    const auto shade = pixel(-25, 0), lit = pixel(25, 0);
    const float inShadow = brightness(a, 100, shade.first, shade.second), inSun = brightness(a, 100, lit.first, lit.second);
    CHECK(inShadow < 0.8f * inSun); // (the filmic curve squeezes bright and dark together)
    // The same picture again (each pixel's samples are seeded by the pixel).
    REQUIRE(renderImage(d.scene, rs, cam, b, nullptr, &why));
    CHECK(a == b);
    // Without the floor the background is the environment: dark, or white.
    rs.floor = false;
    rs.environment = "dark";
    REQUIRE(renderImage(d.scene, rs, cam, b, nullptr, &why));
    CHECK(brightness(b, 100, 5, 5) < 30);
    rs.environment = "white";
    REQUIRE(renderImage(d.scene, rs, cam, b, nullptr, &why));
    CHECK(brightness(b, 100, 5, 5) > 200);
    // Nothing visible: no picture.
    Document empty;
    CHECK(!renderImage(empty.scene, rs, cam, b, nullptr, &why));
    CHECK(!why.empty());
}

TEST_CASE("render: materials side by side: a glowing lamp is the brightest; the command writes the picture") {
    // A block of each, side by side, seen from the front in the dark environment: the lamp
    // is the brightest; the glass shows the lit floor behind; chrome and plastic differ.
    Document d;
    REQUIRE(run(d, "SPHERE 10 @-60,0,0").ok);
    REQUIRE(run(d, "SPHERE 10 @-20,0,0").ok);
    REQUIRE(run(d, "SPHERE 10 @20,0,0").ok);
    REQUIRE(run(d, "SPHERE 10 @60,0,0").ok);
    REQUIRE(run(d, "MATERIAL \"Sphere 1\" chrome").ok);
    REQUIRE(run(d, "MATERIAL \"Sphere 2\" matte COLOR #808080").ok);
    REQUIRE(run(d, "MATERIAL \"Sphere 3\" glass").ok);
    REQUIRE(run(d, "MATERIAL \"Sphere 4\" light COLOR white").ok);
    RenderSettings rs;
    rs.environment = "dark";
    rs.width = 160, rs.height = 40;
    rs.samples = 16;
    rs.floor = false;
    RenderCamera cam;
    cam.position = {0, -300, 10}, cam.forward = {0, 1, 0}, cam.right = {1, 0, 0}, cam.up = {0, 0, 1};
    cam.ortho = true, cam.orthoHeight = 40;
    std::vector<std::uint8_t> px;
    std::string why;
    REQUIRE(renderImage(d.scene, rs, cam, px, nullptr, &why));
    auto at = [&](float wx) { return brightness(px, 160, static_cast<int>((wx / 160.0f + 0.5f) * 160), 20, 1); };
    const float lamp = at(60), matte = at(-20);
    CHECK(lamp > 200);
    CHECK(lamp > matte * 2);
    // Through the command: the 3D view's settings, a picture of the size asked.
    const std::string path = (std::filesystem::temp_directory_path() / "sicdm_render.png").string();
    REQUIRE(run(d, "RENDER \"" + path + "\" SIZE 64 40 SAMPLES 4 ENV studio").ok);
    CHECK(std::filesystem::file_size(path) > 200);
    std::filesystem::remove(path);
    CHECK(run(d, "RENDER").ok); // the settings
    CHECK(!run(d, "RENDER ENV moon").ok);
}

TEST_CASE("materials: metals keep their measured colour, families, finishes and other names") {
    // A metal's colour is stored so the renderer's sRGB-to-linear step gives back its
    // measured reflectance (gold, copper, aluminium...).
    struct Measured {
        const char* name;
        float r, g, b;
    };
    const Measured known[] = {{"gold", 1.000f, 0.766f, 0.336f}, {"copper", 0.955f, 0.638f, 0.538f}, {"aluminium", 0.913f, 0.922f, 0.924f},
                              {"silver", 0.972f, 0.960f, 0.915f}, {"chrome", 0.550f, 0.556f, 0.554f}, {"titanium", 0.542f, 0.497f, 0.449f}};
    for (const Measured& m : known) {
        MaterialProps p;
        Vec3 c{};
        REQUIRE(applyMaterialPreset(m.name, p, &c));
        CHECK(p.metallic == 1.0f);
        CHECK(std::fabs(std::pow(c.x, 2.2f) - m.r) < 0.003f);
        CHECK(std::fabs(std::pow(c.y, 2.2f) - m.g) < 0.003f);
        CHECK(std::fabs(std::pow(c.z, 2.2f) - m.b) < 0.003f);
    }
    // Every preset in a family, names unique, every family used.
    std::vector<std::string> names;
    for (const MaterialPreset& p : materialPresets()) {
        CHECK(std::find(materialFamilies().begin(), materialFamilies().end(), p.family) != materialFamilies().end());
        CHECK(std::find(names.begin(), names.end(), p.name) == names.end());
        names.push_back(p.name);
    }
    for (const char* f : materialFamilies())
        CHECK(std::any_of(materialPresets().begin(), materialPresets().end(), [&](const MaterialPreset& p) { return p.family == f; }));
    CHECK(materialPresets().size() >= 55);
    // Other spellings.
    MaterialProps p;
    Vec3 c{};
    CHECK(applyMaterialPreset("Aluminum", p, &c));
    CHECK(p.preset == "aluminium");
    CHECK(applyMaterialPreset("anodized_red", p, &c));
    CHECK(p.preset == "anodised-red");
    CHECK(applyMaterialPreset("stainless steel", p, &c));
    CHECK(p.preset == "stainless");
    CHECK(!applyMaterialPreset("unobtainium", p, &c));
    // The command: a coloured metal, a finish, the list by family.
    Document d;
    REQUIRE(run(d, "BOX 20 20 20 @0,0,0").ok);
    REQUIRE(run(d, "MATERIAL \"Block 1\" rose-gold").ok);
    CHECK(d.scene.bodies()[0].material.preset == "rose-gold");
    REQUIRE(run(d, "MATERIAL \"Block 1\" FINISH brushed").ok);
    CHECK(std::fabs(d.scene.bodies()[0].material.roughness - 0.45f) < 1e-6f);
    CHECK(d.scene.bodies()[0].material.preset == "rose-gold");
    CHECK(!run(d, "MATERIAL \"Block 1\" FINISH sandy").ok);
    REQUIRE(run(d, "MATERIAL \"Block 1\" anodised COLOR #1E5BD6").ok);
    CHECK(d.scene.bodies()[0].material.metallic == 1.0f);
    CHECK(std::fabs(d.scene.bodies()[0].color.z - 0xD6 / 255.0f) < 1e-3f);
    std::string list;
    for (const std::string& l : run(d, "MATERIAL LIST").lines) list += l + "\n";
    CHECK(list.find("Precious metals:") != std::string::npos);
    CHECK(list.find("platinum") != std::string::npos);
    CHECK(list.find("Finishes") != std::string::npos);
}

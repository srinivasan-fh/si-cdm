# Rendering

![The same parts in the studio environment and under a sky with its sun](render.png)

`RENDER` makes a photoreal picture of the model: metals that mirror their surroundings,
clear glass, glossy paint, soft shadows and contact shadows on a floor. It is a path tracer,
written for SI CDM. Rays from the camera bounce around the scene, and each bounce picks up
light the way real surfaces do.

## Materials

Each body has a material. A new body is glossy plastic in its own colour.

```text
MATERIAL LIST                          the presets
MATERIAL "Bracket" aluminium           a preset (metals and glass bring their own colour)
MATERIAL "Cover" paint COLOR #c0392b   car paint in a colour of your own
MATERIAL "Knob" ROUGHNESS 0.2          one setting at a time
MATERIAL "Lens" glass                  clear: refracts what is behind it
MATERIAL "Screen" light COLOR white    glows
MATERIAL "Head retopo" COLORMAP head_colour.png NORMALMAP head_normal.png
MATERIAL "Bracket" DEFAULT             back to plastic
```

| Setting | Meaning |
| --- | --- |
| `METALLIC` | 0 for plastic, paint or wood; 1 for a metal, which tints its reflections with its colour |
| `ROUGHNESS` | 0 for a mirror finish, 1 for fully matte. Machined metal is about 0.3, polished chrome 0.05. |
| `TRANSMISSION` | 1 for clear (glass, water); refracts by `IOR`: glass 1.5, water 1.33 |
| `EMISSION` | Glows with its colour, this bright |
| `COLORMAP`, `NORMALMAP` | Images on the body's texture coordinates, such as the maps `BAKE` writes (see [sculpting](sculpting.md#uv-unwrapping-and-baking)) |

The presets are plastic, matte, paint, rubber, ceramic, wood, steel, brushed (stainless),
aluminium, anodised, chrome, brass, copper, gold, titanium, glass, tinted (glass in the
body's colour), water and light.

Polypaint shows in the picture too. Materials are saved in the project and undone with
`UNDO`.

## Making the picture

```text
RENDER picture.png                          the 3D view as it is now, with the settings below
RENDER SIZE 1920 1080 SAMPLES 256           a bigger, cleaner picture (the settings are kept)
RENDER ENV sky SUN 135 40                   a sky, the sun at azimuth 135, 40 degrees up
RENDER picture.png VIEW front               from the front (without the app: iso by default)
RENDER                                      the settings now
```

| Setting | Default | Meaning |
| --- | --- | --- |
| `ENV` | studio | `studio`: a photo studio's soft boxes (key, fill, rim and top light); `sky`: a blue sky and its sun; `white`: even white light; `dark`: soft boxes in a dark room |
| `SUN az el` | 135 45 | `sky`: where the sun is, in degrees |
| `LIGHT` | 1 | How bright the environment is |
| `EXPOSURE` | 0 | Stops brighter (+) or darker (−) |
| `FLOOR` | ON | A floor under the bodies, for contact shadows and reflections. It fades into the environment far away. |
| `SIZE w h` | 1280 800 | Pixels |
| `SAMPLES` | 64 | Rays a pixel. More gives less noise and takes longer: 16 for a preview, 256 or more for a final picture. |
| `BOUNCES` | 6 | How many times light may bounce (glass and mirrors need a few) |
| `DENOISE` | ON | Takes out the remaining noise, keeping edges, textures and paint sharp |

The settings are saved in the project.

**In the app:** *View → Render...* opens the render window.

![The render window](render-window.png)

- **Settings:** the environment, light, exposure, floor, size and samples.
- **Material:** the material of the selected body, with its roughness and metallic sliders.
- **Render:** makes the picture in the background from the 3D view as it is, with a progress
  bar and *Stop*.
- **Save Picture:** writes the result.

**MCP:** the `render` tool sets a body's material and makes the picture in one call.

## How it works

- **Surfaces:**
  - Diffuse colour plus glossy reflection from microfacets (GGX), with Fresnel. Metals tint
    their reflection with their own colour.
  - Glass reflects or refracts by the exact Fresnel term.
- **Light:** the environment's lights (soft boxes, the sun) are aimed at directly from every
  bounce, as well as found by the bounces themselves. Weighting the two by how likely each was
  (multiple importance sampling) makes small bright lights and mirror-like surfaces both clean.
- **Noise:** each pixel's spread of samples is measured. An edge-aware filter, guided by the
  surfaces' colours, normals and distances, smooths only the light, and only within each
  surface.
- **Tone:** a filmic curve (ACES), then sRGB.
- **Speed:** a few seconds for a preview on a modern laptop. It uses every core and the
  bounding-volume hierarchy used for picking.
- **Repeatable:** the same scene and settings always give the same picture.

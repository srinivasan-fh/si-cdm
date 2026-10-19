#!/usr/bin/env python3
"""Renders the SI CDM app icon and writes every size the platforms need.

    python3 tools/make_icon.py

Outputs (checked in, so building needs no Python):
  resources/icon/sicdm_1024.png        master artwork
  resources/icon/sicdm_{256,64,32}.png window icons (embedded into the app)
  resources/macos/SICDM.icns           macOS bundle icon (Dock, Finder)

Design: a macOS-style rounded square holding deep space, the glowing rim of a
Mars-like planet, an isometric CAD cube in the app's accent blue and an orbit
ring (simulation) with a small moon.
"""
import os
import struct
import io

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SS = 4                      # supersampling
N = 1024 * SS               # working canvas


def lerp(a, b, t):
    return a + (b - a) * t


def squircle_mask(size, inset, n=5.0):
    """macOS-like superellipse: |x|^n + |y|^n <= 1 inside the inset square."""
    ys, xs = np.mgrid[0:size, 0:size].astype(np.float32)
    half = (size - 2 * inset) / 2.0
    c = size / 2.0
    u = np.abs((xs + 0.5 - c) / half)
    v = np.abs((ys + 0.5 - c) / half)
    d = u ** n + v ** n
    # Soft edge over ~1.5 output pixels.
    edge = 1.5 * SS / half * n
    return np.clip((1.0 - d) / edge + 0.5, 0.0, 1.0)


def background():
    ys, xs = np.mgrid[0:N, 0:N].astype(np.float32) / N
    top = np.array([12, 18, 34], np.float32)
    bottom = np.array([28, 42, 74], np.float32)
    img = lerp(top, bottom, ys[..., None] ** 1.4)
    # Faint nebula glow upper left.
    g = np.exp(-(((xs - 0.28) ** 2 + (ys - 0.30) ** 2) / 0.05))[..., None]
    img = img + g * np.array([40, 30, 90], np.float32) * 0.55
    # Stars.
    rng = np.random.default_rng(7)
    for _ in range(140):
        x, y = rng.uniform(0.12, 0.88), rng.uniform(0.1, 0.62)
        r = rng.uniform(0.6, 2.2) * SS
        b = rng.uniform(0.35, 1.0)
        xi, yi = int(x * N), int(y * N)
        rr = int(r * 3) + 1
        sub_y, sub_x = np.mgrid[-rr:rr + 1, -rr:rr + 1].astype(np.float32)
        s = np.exp(-(sub_x ** 2 + sub_y ** 2) / (r * r))[..., None] * b * 255
        y0, y1, x0, x1 = yi - rr, yi + rr + 1, xi - rr, xi + rr + 1
        img[y0:y1, x0:x1] = np.maximum(img[y0:y1, x0:x1], s * np.array([0.9, 0.95, 1.0]))
    return img


def planet(img):
    """Mars-like planet rising from the bottom, lit from the upper right."""
    ys, xs = np.mgrid[0:N, 0:N].astype(np.float32) / N
    cx, cy, r = 0.5, 1.52, 0.78
    d = np.sqrt((xs - cx) ** 2 + (ys - cy) ** 2)
    inside = np.clip((r - d) * N / (1.5 * SS), 0, 1)[..., None]
    # Surface: rust gradient + bands, terminator shading.
    lit = np.clip(((xs - cx) * 0.5 + (cy - ys) * 0.9) / r * 1.2 + 0.25, 0, 1)
    band = 0.5 + 0.5 * np.sin(ys * 90.0 + np.sin(xs * 14.0) * 2.0)
    base = lerp(np.array([92, 34, 18], np.float32), np.array([214, 104, 52], np.float32), lit[..., None])
    base = base * (0.9 + 0.1 * band[..., None])
    img = img * (1 - inside) + base * inside
    # Atmosphere rim glow.
    rim = np.exp(-((d - r) ** 2) / (0.00018)) * (0.35 + 0.65 * np.clip((xs - 0.15) / 0.7, 0, 1))
    img = img + rim[..., None] * np.array([255, 150, 90], np.float32) * 0.9
    halo = np.exp(-np.clip(d - r, 0, None) / 0.035) * (d > r)
    img = img + halo[..., None] * np.array([255, 120, 60], np.float32) * 0.22
    return img


def iso(x, y, z, scale, ox, oy):
    """Isometric projection, Z up."""
    px = (x - y) * 0.8660254
    py = (x + y) * 0.5 - z
    return ox + px * scale, oy + py * scale


def cube_layer():
    layer = Image.new("RGBA", (N, N), (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    s, ox, oy = 0.205 * N, 0.5 * N, 0.505 * N
    P = {k: iso(*v, s, ox, oy) for k, v in {
        "a": (-1, -1, 1), "b": (1, -1, 1), "c": (1, 1, 1), "e": (-1, 1, 1),
        "f": (-1, 1, -1), "g": (1, 1, -1), "h": (1, -1, -1)}.items()}
    top = [P["a"], P["b"], P["c"], P["e"]]
    left = [P["e"], P["c"], P["g"], P["f"]]
    right = [P["c"], P["b"], P["h"], P["g"]]
    d.polygon(top, fill=(132, 196, 255, 255))
    d.polygon(left, fill=(40, 108, 214, 255))
    d.polygon(right, fill=(22, 66, 150, 255))
    # Engraved inner square on the top face (a CAD sketch), and a hole.
    inner = [iso(x * 0.55, y * 0.55, 1, s, ox, oy) for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    d.line(inner + [inner[0]], fill=(235, 246, 255, 255), width=int(0.007 * N), joint="curve")
    cx, cy = iso(0, 0, 1, s, ox, oy)
    rx, ry = 0.30 * s * 0.8660254 * 2 * 0.55, 0.30 * s * 0.5 * 2 * 0.55
    d.ellipse([cx - rx, cy - ry, cx + rx, cy + ry], fill=(30, 84, 180, 255))
    # Crisp edges.
    w = int(0.006 * N)
    for poly in (top, left, right):
        d.line(poly + [poly[0]], fill=(225, 240, 255, 230), width=w, joint="curve")
    return layer


def orbit_layer(front):
    """Tilted orbit ring; the front half is drawn over the cube."""
    layer = Image.new("RGBA", (N, N), (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    cx, cy, rx, ry = 0.5 * N, 0.50 * N, 0.40 * N, 0.13 * N
    import math
    pts = []
    for i in range(721):
        t = i / 720 * 2 * math.pi
        x = cx + rx * math.cos(t)
        y = cy + ry * math.sin(t)
        # Rotate the ellipse by -14 degrees.
        a = math.radians(-14)
        xr = cx + (x - cx) * math.cos(a) - (y - cy) * math.sin(a)
        yr = cy + (x - cx) * math.sin(a) + (y - cy) * math.cos(a)
        pts.append((xr, yr, math.sin(t)))
    w = int(0.012 * N)
    for i in range(len(pts) - 1):
        (x0, y0, s0), (x1, y1, _) = pts[i], pts[i + 1]
        if (s0 > 0) == front:
            alpha = 255 if front else 150
            d.line([(x0, y0), (x1, y1)], fill=(255, 196, 120, alpha), width=w)
    if front:
        # The moon on the front of the orbit.
        mx, my, _ = pts[95]
        r = 0.035 * N
        d.ellipse([mx - r, my - r, mx + r, my + r], fill=(255, 226, 180, 255))
    return layer


def render():
    img = planet(background())
    rgb = Image.fromarray(np.clip(img, 0, 255).astype(np.uint8), "RGB").convert("RGBA")
    back = orbit_layer(False).filter(ImageFilter.GaussianBlur(SS * 0.6))
    glow = orbit_layer(True).filter(ImageFilter.GaussianBlur(SS * 10))
    rgb = Image.alpha_composite(rgb, back)
    # Cube with a soft shadow / glow underneath.
    cube = cube_layer()
    shadow = cube.split()[3].filter(ImageFilter.GaussianBlur(SS * 22))
    shade = Image.new("RGBA", (N, N), (6, 10, 24, 0))
    shade.putalpha(shadow.point(lambda a: int(a * 0.75)))
    rgb = Image.alpha_composite(rgb, shade)
    rgb = Image.alpha_composite(rgb, cube)
    rgb = Image.alpha_composite(rgb, glow)
    rgb = Image.alpha_composite(rgb, orbit_layer(True))
    # Subtle glass highlight across the top.
    ys = np.mgrid[0:N, 0:N][0].astype(np.float32) / N
    hl = (np.clip(1 - ys / 0.45, 0, 1) ** 2 * 26).astype(np.uint8)
    hi = Image.new("RGBA", (N, N), (255, 255, 255, 0))
    hi.putalpha(Image.fromarray(hl, "L"))
    rgb = Image.alpha_composite(rgb, hi)
    # Squircle mask (macOS grid: 824 px tile inside 1024 with margins).
    mask = squircle_mask(N, inset=100 * SS)
    a = np.array(rgb.split()[3], np.float32) * mask
    rgb.putalpha(Image.fromarray(a.astype(np.uint8), "L"))
    # Drop shadow under the tile.
    sh = Image.fromarray((mask * 110).astype(np.uint8), "L").filter(ImageFilter.GaussianBlur(SS * 14))
    shadow_img = Image.new("RGBA", (N, N), (0, 0, 0, 0))
    shadow_img.putalpha(sh)
    shadow_img = shadow_img.transform((N, N), Image.AFFINE, (1, 0, 0, 0, 1, -10 * SS))
    out = Image.alpha_composite(shadow_img, rgb)
    return out.resize((1024, 1024), Image.LANCZOS)


def write_icns(master, path):
    entries = [(b"ic11", 32), (b"ic12", 64), (b"ic07", 128), (b"ic13", 256), (b"ic08", 256),
               (b"ic14", 512), (b"ic09", 512), (b"ic10", 1024)]
    chunks = []
    for tag, size in entries:
        buf = io.BytesIO()
        master.resize((size, size), Image.LANCZOS).save(buf, "PNG", optimize=True)
        data = buf.getvalue()
        chunks.append(tag + struct.pack(">I", len(data) + 8) + data)
    body = b"".join(chunks)
    with open(path, "wb") as f:
        f.write(b"icns" + struct.pack(">I", len(body) + 8) + body)


def main():
    master = render()
    icon_dir = os.path.join(ROOT, "resources", "icon")
    os.makedirs(icon_dir, exist_ok=True)
    master.save(os.path.join(icon_dir, "sicdm_1024.png"), optimize=True)
    for size in (256, 64, 32):
        master.resize((size, size), Image.LANCZOS).save(os.path.join(icon_dir, f"sicdm_{size}.png"), optimize=True)
    write_icns(master, os.path.join(ROOT, "resources", "macos", "SICDM.icns"))
    print("icons written")


if __name__ == "__main__":
    main()

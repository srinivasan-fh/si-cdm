#!/usr/bin/env python3
"""Renders the SI CDM app icon and writes every size the platforms need.

    python3 tools/make_icon.py

Outputs (checked in, so building needs no Python):
  resources/icon/sicdm_1024.png        master artwork
  resources/icon/sicdm_{256,64,32}.png window icons (embedded into the app)
  resources/macos/SICDM.icns           macOS bundle icon (Dock, Finder)
  resources/windows/sicdm.ico          Windows executable and installer icon

Design: a macOS-style rounded square holding the splash screen's scene: deep
space with stars and a crescent Earth, the curved horizon of the Moon with
craters, and the crew rover (light panels, black tyres on gold dish rims,
glowing headlamps with a lens streak) lit by a low sun from the right.
"""
import io
import math
import os
import struct

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


def P(x, y):
    return (x * N, y * N)


def poly(d, pts, fill):
    d.polygon([P(x, y) for x, y in pts], fill=fill)


def ellipse(d, cx, cy, rx, ry, fill):
    d.ellipse([(cx - rx) * N, (cy - ry) * N, (cx + rx) * N, (cy + ry) * N], fill=fill)


def layer():
    img = Image.new("RGBA", (N, N), (0, 0, 0, 0))
    return img, ImageDraw.Draw(img)


def glow(x, y, sigma, color, strength, sx=1.0, sy=1.0):
    """Additive gaussian glow as a float RGB array (sx / sy stretch it)."""
    ys, xs = np.mgrid[0:N, 0:N].astype(np.float32) / N
    g = np.exp(-(((xs - x) / sx) ** 2 + ((ys - y) / sy) ** 2) / (sigma * sigma))
    return g[..., None] * np.array(color, np.float32) * strength


def background():
    ys, xs = np.mgrid[0:N, 0:N].astype(np.float32) / N
    top = np.array([4, 6, 12], np.float32)
    bottom = np.array([14, 20, 36], np.float32)
    img = lerp(top, bottom, np.clip(ys / 0.65, 0, 1)[..., None] ** 1.6)
    # Faint Milky Way band, diagonal.
    band = np.exp(-(((xs - ys * 0.9) - 0.12) ** 2) / 0.012)
    img = img + band[..., None] * np.array([30, 34, 52], np.float32) * 0.5
    rng = np.random.default_rng(11)
    for _ in range(170):
        x, y = rng.uniform(0.1, 0.9), rng.uniform(0.09, 0.62)
        r = rng.uniform(0.5, 1.9) * SS
        b = rng.uniform(0.3, 1.0) ** 1.5
        xi, yi = int(x * N), int(y * N)
        rr = int(r * 3) + 1
        sub_y, sub_x = np.mgrid[-rr:rr + 1, -rr:rr + 1].astype(np.float32)
        tint = np.array([1.0, 0.92, 0.85]) if rng.uniform() < 0.4 else np.array([0.85, 0.92, 1.0])
        st = np.exp(-(sub_x ** 2 + sub_y ** 2) / (r * r))[..., None] * b * 255 * tint
        y0, y1, x0, x1 = yi - rr, yi + rr + 1, xi - rr, xi + rr + 1
        img[y0:y1, x0:x1] = np.maximum(img[y0:y1, x0:x1], st)
    return img


def earth(img):
    """Crescent Earth, lit from the right, with a blue limb."""
    ys, xs = np.mgrid[0:N, 0:N].astype(np.float32) / N
    cx, cy, r = 0.345, 0.2, 0.062
    dx, dy = (xs - cx) / r, (ys - cy) / r
    rr = dx * dx + dy * dy
    inside = np.clip((1 - np.sqrt(rr)) * r * N / (1.2 * SS), 0, 1)
    nz = np.sqrt(np.clip(1 - rr, 0, 1))
    lit = np.clip((dx * 0.93 - dy * 0.2 + nz * -0.3) * 1.6 + 0.15, 0, 1)
    rng = np.random.default_rng(3)
    surf = np.zeros((N, N, 3), np.float32) + np.array([18, 60, 150], np.float32)
    # Continents and cloud swirls from a few soft blobs.
    for _ in range(14):
        bx, by, br = rng.uniform(-0.8, 0.9), rng.uniform(-0.9, 0.9), rng.uniform(0.12, 0.35)
        blob = np.exp(-((dx - bx) ** 2 + (dy - by) ** 2) / (br * br))
        surf = lerp(surf, np.array([70, 92, 52], np.float32), np.clip(blob * 1.6 - 0.5, 0, 1)[..., None])
    for _ in range(18):
        bx, by = rng.uniform(-0.9, 1.0), rng.uniform(-0.9, 0.9)
        bw, bh = rng.uniform(0.25, 0.6), rng.uniform(0.1, 0.22)
        blob = np.exp(-(((dx - bx) / bw) ** 2 + ((dy - by) / bh) ** 2))
        surf = lerp(surf, np.array([245, 248, 255], np.float32), np.clip(blob * 1.1 - 0.35, 0, 0.8)[..., None])
    col = surf * (lit[..., None] * 1.15) + np.array([2, 3, 8], np.float32)
    limb = (1 - nz) ** 3 * np.clip(lit * 1.4, 0, 1)
    col = col + limb[..., None] * np.array([90, 150, 255], np.float32) * 0.9
    img = img * (1 - inside[..., None]) + col * inside[..., None]
    d = np.sqrt(rr)
    halo = np.exp(-np.clip(d - 1, 0, None) / 0.12) * (d > 1) * np.clip(dx * 0.9 + 0.3, 0, 1)
    return img + halo[..., None] * np.array([60, 110, 255], np.float32) * 0.35


def moon(img):
    """The lunar surface: a huge sphere whose horizon curves across the lower third."""
    ys, xs = np.mgrid[0:N, 0:N].astype(np.float32) / N
    cx, cy, r = 0.5, 1.86, 1.24
    d = np.sqrt((xs - cx) ** 2 + (ys - cy) ** 2)
    inside = np.clip((r - d) * N / (1.2 * SS), 0, 1)
    depth = np.clip((ys - (cy - r)) / 0.35, 0, 1)            # 0 at the horizon, 1 near us
    lit = np.clip(0.55 + (xs - 0.5) * 0.7, 0.2, 1.0)
    base = lerp(np.array([150, 150, 154], np.float32), np.array([96, 96, 102], np.float32), depth[..., None])
    col = base * lit[..., None]
    rng = np.random.default_rng(5)
    noise = rng.normal(0, 1, (N // 8, N // 8)).astype(np.float32)
    noise = np.array(Image.fromarray(noise).resize((N, N), Image.BICUBIC))
    col = col * (1 + noise[..., None] * 0.05)
    # Craters: dark bowls with a lit rim on the sunward (right) side, flattened by perspective.
    for _ in range(34):
        x = rng.uniform(0.05, 0.95)
        y = rng.uniform(0.66, 0.97)
        ry = rng.uniform(0.006, 0.022) * (0.4 + 1.6 * (y - 0.62))
        rx = ry * rng.uniform(2.6, 3.4)
        if np.hypot(x - cx, y - cy) > r - ry:
            continue
        ex, ey = (xs - x) / rx, (ys - y) / ry
        e = ex * ex + ey * ey
        bowl = np.clip(1 - e, 0, 1) ** 0.5
        shade = np.clip(-ex * 0.8 + 0.3, 0, 1)                # far wall (left) dark, right wall lit
        col = col * (1 - (bowl * shade * 0.55)[..., None])
        rim = np.exp(-((np.sqrt(e) - 1) ** 2) / 0.02) * np.clip(ex, 0, 1)
        col = col + rim[..., None] * np.array([60, 60, 62], np.float32)
    img = img * (1 - inside[..., None]) + col * inside[..., None]
    # Bright sunlit edge of the horizon.
    edge = np.exp(-((d - r) ** 2) / 0.00002) * np.clip((xs - 0.1) / 0.8, 0, 1)
    return img + edge[..., None] * np.array([200, 200, 205], np.float32) * 0.5


# Rover geometry in icon units (0..1). It faces front-right, seen from front-left.
GROUND = 0.745


def wheel(d, cx, cy, r, squash, far=False):
    """Tyre with tread blocks and a gold dish rim, seen at an angle (x squashed)."""
    rx = r * squash
    tread = (16, 16, 18, 255)
    ellipse(d, cx, cy, rx, r, tread)
    for i in range(26):                                         # tread blocks on the rim
        a = i / 26 * 2 * math.pi
        ca, sa = math.cos(a), math.sin(a)
        t0, t1, hw = 0.97, 1.07, 0.06
        corners = [(cx + math.cos(a + o) * rx * k, cy + math.sin(a + o) * r * k)
                   for k, o in ((t0, -hw), (t1, -hw * 0.8), (t1, hw * 0.8), (t0, hw))]
        poly(d, corners, (12, 12, 14, 255))
    ellipse(d, cx, cy, rx * 0.92, r * 0.92, (30, 30, 33, 255))   # sidewall
    if far:
        ellipse(d, cx, cy, rx * 0.6, r * 0.6, (110, 78, 34, 255))
        ellipse(d, cx, cy, rx * 0.5, r * 0.5, (84, 60, 28, 255))
        return
    ellipse(d, cx, cy, rx * 0.64, r * 0.64, (214, 160, 78, 255)) # rim lip
    ellipse(d, cx - rx * 0.03, cy, rx * 0.55, r * 0.55, (150, 104, 44, 255))  # dish (shadowed)
    ellipse(d, cx + rx * 0.08, cy - r * 0.05, rx * 0.42, r * 0.42, (196, 142, 62, 255))
    ellipse(d, cx, cy, rx * 0.17, r * 0.17, (64, 64, 70, 255))   # hub
    for i in range(5):
        a = i / 5 * 2 * math.pi + 0.3
        ellipse(d, cx + math.cos(a) * rx * 0.1, cy + math.sin(a) * r * 0.1, 0.006 * squash, 0.006, (150, 150, 158, 255))
    # Rim highlight from the sun (upper right).
    d.arc([(cx - rx * 0.63) * N, (cy - r * 0.63) * N, (cx + rx * 0.63) * N, (cy + r * 0.63) * N],
          start=-80, end=20, fill=(255, 220, 150, 255), width=int(0.006 * N))


def rover_layers():
    body, d = layer()
    # Far-side front wheel peeking out past the nose.
    wheel(d, 0.80, 0.645, 0.088, 0.42, far=True)
    # Chassis (dark), side panel (light, in partial shade), nose (sunlit).
    poly(d, [(0.19, 0.585), (0.72, 0.60), (0.83, 0.585), (0.83, 0.625), (0.72, 0.645), (0.19, 0.625)], (24, 25, 29, 255))
    poly(d, [(0.17, 0.455), (0.27, 0.405), (0.52, 0.395), (0.66, 0.43), (0.72, 0.47), (0.72, 0.60), (0.18, 0.585)],
         (128, 131, 136, 255))
    poly(d, [(0.72, 0.47), (0.86, 0.455), (0.87, 0.585), (0.72, 0.60)], (196, 199, 203, 255))
    # Hood top and roof (catch the sun), raked windscreen (dark glass).
    poly(d, [(0.66, 0.43), (0.80, 0.42), (0.86, 0.455), (0.72, 0.47)], (222, 224, 227, 255))
    poly(d, [(0.27, 0.405), (0.40, 0.37), (0.56, 0.365), (0.52, 0.395)], (176, 179, 184, 255))
    poly(d, [(0.52, 0.395), (0.56, 0.365), (0.69, 0.37), (0.80, 0.42), (0.66, 0.43)], (14, 17, 24, 255))
    poly(d, [(0.585, 0.372), (0.66, 0.375), (0.72, 0.40), (0.63, 0.40)], (60, 70, 90, 255))  # glass reflection
    # Side window band and panel seams.
    poly(d, [(0.29, 0.43), (0.55, 0.425), (0.62, 0.45), (0.62, 0.48), (0.29, 0.485)], (18, 20, 26, 255))
    for x in (0.38, 0.50):
        d.line([P(x, 0.495), P(x, 0.585)], fill=(96, 99, 104, 255), width=int(0.004 * N))
    d.line([P(0.19, 0.52), P(0.72, 0.53)], fill=(104, 107, 112, 255), width=int(0.004 * N))
    # Dark front fascia with two headlamp housings.
    poly(d, [(0.735, 0.505), (0.865, 0.495), (0.868, 0.565), (0.738, 0.575)], (20, 21, 25, 255))
    # Rear module, roof rack, antenna, dish, mast camera.
    poly(d, [(0.15, 0.46), (0.19, 0.44), (0.19, 0.60), (0.15, 0.595)], (40, 42, 47, 255))
    d.line([P(0.31, 0.385), P(0.50, 0.375)], fill=(70, 72, 78, 255), width=int(0.008 * N))
    d.line([P(0.215, 0.425), P(0.215, 0.29)], fill=(150, 152, 158, 255), width=int(0.0045 * N))
    ellipse(d, 0.215, 0.287, 0.006, 0.006, (220, 222, 226, 255))
    d.line([P(0.37, 0.38), P(0.37, 0.345)], fill=(120, 122, 128, 255), width=int(0.006 * N))
    d.chord([(0.32) * N, (0.30) * N, (0.42) * N, (0.36) * N], start=160, end=340, fill=(205, 208, 212, 255))
    d.line([P(0.62, 0.37), P(0.62, 0.31)], fill=(120, 122, 128, 255), width=int(0.006 * N))
    poly(d, [(0.60, 0.29), (0.65, 0.29), (0.655, 0.315), (0.60, 0.315)], (30, 31, 36, 255))
    ellipse(d, 0.65, 0.302, 0.006, 0.007, (90, 140, 220, 255))
    # Near-side wheels with fenders.
    for cx, cy, r in ((0.295, 0.648, 0.098), (0.615, 0.655, 0.105)):
        d.arc([(cx - r * 1.15 * 0.78) * N, (cy - r * 1.15) * N, (cx + r * 1.15 * 0.78) * N, (cy + r * 1.15) * N],
              start=190, end=350, fill=(34, 35, 40, 255), width=int(0.022 * N))
        wheel(d, cx, cy, r, 0.78)

    # Sunlit edges.
    w = int(0.0045 * N)
    d.line([P(0.27, 0.405), P(0.40, 0.37), P(0.56, 0.365)], fill=(236, 238, 241, 255), width=w, joint="curve")
    d.line([P(0.66, 0.43), P(0.80, 0.42), P(0.86, 0.455), P(0.87, 0.585)], fill=(246, 247, 249, 255), width=w,
           joint="curve")
    d.line([P(0.72, 0.47), P(0.72, 0.60)], fill=(160, 163, 168, 255), width=w)

    lamps = [(0.765, 0.535), (0.79, 0.533), (0.835, 0.53), (0.858, 0.529)]
    for x, y in lamps:
        ellipse(d, x, y, 0.0105, 0.012, (255, 236, 200, 255))
    return body, lamps


def render():
    img = moon(earth(background()))
    # Rover shadow on the ground (sun from the right: shadow falls to the left).
    ys, xs = np.mgrid[0:N, 0:N].astype(np.float32) / N
    sh = np.exp(-(((xs - 0.43) / 0.36) ** 2 + ((ys - (GROUND - 0.005)) / 0.022) ** 2) * 2.2)
    img = img * (1 - sh[..., None] * 0.8)
    rgb = Image.fromarray(np.clip(img, 0, 255).astype(np.uint8), "RGB").convert("RGBA")
    body, lamps = rover_layers()
    # Shade the rover darker towards the ground.
    b = np.array(body, np.float32)
    shade = np.clip(1.12 - (ys - 0.38) * 0.9, 0.7, 1.12)
    b[..., :3] = np.clip(b[..., :3] * shade[..., None], 0, 255)
    body = Image.fromarray(b.astype(np.uint8), "RGBA")
    rgb = Image.alpha_composite(rgb, body)
    # Headlamp glow and an anamorphic streak (additive).
    add = np.zeros((N, N, 3), np.float32)
    for x, y in lamps:
        add += glow(x, y, 0.016, (255, 214, 160), 0.9)
        add += glow(x, y, 0.05, (255, 190, 120), 0.18)
    add += glow(0.81, 0.532, 0.012, (150, 190, 255), 0.55, sx=22.0, sy=0.45)
    out = np.array(rgb, np.float32)
    out[..., :3] = np.clip(out[..., :3] + add, 0, 255)
    rgb = Image.fromarray(out.astype(np.uint8), "RGBA")
    # Subtle glass highlight across the top.
    hl = (np.clip(1 - ys / 0.45, 0, 1) ** 2 * 18).astype(np.uint8)
    hi = Image.new("RGBA", (N, N), (255, 255, 255, 0))
    hi.putalpha(Image.fromarray(hl, "L"))
    rgb = Image.alpha_composite(rgb, hi)
    # Squircle mask (macOS grid: 824 px tile inside 1024 with margins).
    mask = squircle_mask(N, inset=100 * SS)
    a = np.array(rgb.split()[3], np.float32) * mask
    rgb.putalpha(Image.fromarray(a.astype(np.uint8), "L"))
    # Drop shadow under the tile.
    shm = Image.fromarray((mask * 110).astype(np.uint8), "L").filter(ImageFilter.GaussianBlur(SS * 14))
    shadow_img = Image.new("RGBA", (N, N), (0, 0, 0, 0))
    shadow_img.putalpha(shm)
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
    master.resize((256, 256), Image.LANCZOS).save(os.path.join(ROOT, "resources", "windows", "sicdm.ico"),
                                                  sizes=[(n, n) for n in (16, 24, 32, 48, 64, 128, 256)])
    print("icons written")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Writes docs/examples/suv.txt: a full-size luxury SUV exterior as SI-CDM commands.

    tools/suv.py > docs/examples/suv.txt

Sizes (mm): 5052 long, 2047 wide, 1870 high, 2997 wheelbase, 230 ground clearance, 285/45 R22
tyres (815 across). Built nose at x = 0 (Y across, Z up) from the tables below, then mirrored
so the nose points +X. An upright body with near-vertical sides and a clamshell bonnet; a dark
glasshouse with a flat roof (a "floating" roof); slim lamps, a dark grille; big wheels.
"""
import math

L, W, H, WB = 5052.0, 2047.0, 1870.0, 2997.0
FA = 930.0; RA = FA + WB
TYRE_R, TYRE_W, RIM_R = 408.0, 285.0, 279.0

def interp(table, x):
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x0 <= x <= x1:
            t = (x - x0) / (x1 - x0); t = t * t * (3 - 2 * t)
            return y0 + (y1 - y0) * t
    return table[-1][1] if x > table[-1][0] else table[0][1]

# Lines along x: bonnet / belt / deck top, bonnet dip, underbody, half width, roof.
top  = [(0, 930), (60, 985), (300, 1015), (930, 1040), (1500, 1075), (3000, 1100), (4500, 1112), (4960, 1105), (5052, 1040)]
dip  = [(0, 6), (600, 10), (1400, 6), (1600, 0), (5052, 0)]
bot  = [(0, 440), (200, 330), (650, 263), (930, 243), (2500, 237), (3927, 247), (4500, 307), (5052, 483)]
half = [(0, 900), (80, 975), (400, 1008), (930, 1021), (2500, 1019), (3927, 1021), (4700, 1005), (4980, 954), (5052, 899)]
G0, G1 = 1520.0, 4975.0
roof = [(G0, 1065), (1900, 1555), (2250, 1828), (2500, 1853), (4300, 1857), (4760, 1835), (4930, 1630), (G1, 1125)]

def section(x, k=1.0):
    hw, zb, zt = interp(half, x), interp(bot, x), interp(top, x)
    if k != 1.0:
        zc = 0.5 * (zb + zt); hw *= k; zb = zc - (zc - zb) * k; zt = zc + (zt - zc) * k
    h = zt - zb; d = interp(dip, x) * k
    pts = [(0, zb), (0.82 * hw, zb), (0.95 * hw, zb + 0.035 * h), (0.993 * hw, zb + 0.13 * h), (hw, zb + 0.42 * h),
           (0.997 * hw, zb + 0.78 * h), (0.982 * hw, zt - 0.035 * h), (0.93 * hw, zt), (0.50 * hw, zt - 0.6 * d), (0, zt - d)]
    return pts + [(-y, z) for (y, z) in reversed(pts[1:-1])]

def gsection(x):
    zr = interp(roof, x); base = interp(top, x) - 25
    taper = min(1.0, (G1 - x) / 160.0, (x - G0) / 200.0 + 0.7)
    hb = (interp(half, x) - 118) * (0.92 + 0.08 * max(taper, 0.0)); hr = hb * 0.87
    h = max(zr - base, 40.0)
    pts = [(0, base), (hb, base), (hb * 0.99, base + 0.40 * h), (hr * 1.0, base + 0.93 * h), (hr * 0.88, base + h * 0.995), (0, base + h)]
    return pts + [(-y, z) for (y, z) in reversed(pts[1:-1])]

def spline(pts): return "SPLINE " + " ".join("%.1f,%.1f" % p for p in pts) + " CLOSE"
def pline(pts): return "PLINE " + " ".join("%.2f,%.2f" % p for p in pts) + " CLOSE"

def tyre_section(y0, y1, z0, z1, r_out, r_in, n=8):
    pts = []
    def arc(cy, cz, r, a0, a1):
        for i in range(n + 1):
            a = a0 + (a1 - a0) * i / n
            pts.append((cy + r * math.cos(a), cz + r * math.sin(a)))
    arc(y1 - r_out, z1 - r_out, r_out, 0, math.pi / 2)
    arc(y0 + r_out, z1 - r_out, r_out, math.pi / 2, math.pi)
    arc(y0 + r_in, z0 + r_in, r_in, math.pi, 1.5 * math.pi)
    arc(y1 - r_in, z0 + r_in, r_in, 1.5 * math.pi, 2 * math.pi)
    return pts

def wheel(tag, xc, yc, zc, outboard):
    s = 1 if outboard > 0 else -1
    k = RIM_R / 228.0; w = TYRE_W / 235.0  # the sedan's rim drawn bigger
    out = ["PLANE RIGHT", pline(tyre_section(-TYRE_W / 2, TYRE_W / 2, RIM_R - 4, TYRE_R, 60, 12)),
           "REVOLVE AXIS -400,0 400,0 ANGLE 360", "RENAME Tyre%s" % tag]
    prof = [(-112 * w, RIM_R - 16), (-112 * w, RIM_R), (112 * w, RIM_R), (120 * w, RIM_R - 8), (120 * w, 48 * k), (96 * w, 48 * k), (96 * w, RIM_R - 26)]
    out += [pline([(y * s, z) for (y, z) in prof]), "REVOLVE AXIS -400,0 400,0 ANGLE 360", "RENAME Rim%s" % tag]
    # Ten slim windows: a ten-spoke look.
    out += ["CYLINDER %.0f 500 @0,0,0" % (34 * k), "RENAME Win%s" % tag, "ROTATE Win%s 90,0,0" % tag,
            "MOVE 0,250,%.1f Win%s" % (150 * k, tag), 'PATTERN Win%s CIRCULAR 10 AXIS Y AT 0,0,0 SUBTRACT "Rim%s"' % (tag, tag)]
    out += ["CYLINDER %.0f 30 @0,0,0" % (172 * k), "RENAME Disc%s" % tag, "ROTATE Disc%s 90,0,0" % tag, "MOVE 0,%.1f,0 Disc%s" % (s * 80 * w + 15, tag)]
    out += ["BOX 90 40 150 @0,0,0", "RENAME Caliper%s" % tag, "MOVE %.1f,%.1f,%.1f Caliper%s" % (-200 if xc < 2000 else 200, s * 66 * w, -75, tag)]
    for p in ("Tyre", "Rim", "Disc", "Caliper"): out.append("MOVE %.1f,%.1f,%.1f %s%s" % (xc, yc, zc, p, tag))
    out += ["MATERIAL Tyre%s rubber" % tag, "MATERIAL Rim%s aluminium" % tag, "MATERIAL Rim%s COLOR #8e9196" % tag, "MATERIAL Rim%s FINISH satin" % tag,
            "MATERIAL Disc%s steel" % tag, "MATERIAL Disc%s COLOR #4a4d52" % tag, "MATERIAL Caliper%s COLOR #1c1d20" % tag]
    return out

out = ["NEW", "UNITS mm", "PLANE RIGHT"]
stations = [0, 50, 160, 420, 930, 1500, 2200, 3000, 3927, 4500, 4850, 4980, 5052]
for x in stations:
    out.append(spline(section(50 if x == 0 else 4980 if x == L else x, 0.84 if x == 0 else 0.88 if x == L else 1.0)))
out.append("LOFT " + " ".join('"Spline %d" AT %.0f' % (i + 1, x) for i, x in enumerate(stations)) + " SMOOTH")
out.append('RENAME "Loft 1" Body')
gst = [G0 + 1, 1700, 1900, 2250, 2500, 3300, 4300, 4760, 4930, G1 - 1]
k0 = len(stations)
for x in gst: out.append(spline(gsection(x)))
out.append("LOFT " + " ".join('"Spline %d" AT %.0f' % (k0 + i + 1, x) for i, x in enumerate(gst)) + " SMOOTH")
out.append('RENAME "Loft 2" Glasshouse')

def cut(make, tool, place):
    out.extend(make); out.append("RENAME %s" % tool); out.extend(place); out.append("BOOLEAN Body SUBTRACT %s" % tool)
# Wheel arches (a little bigger than the tyres, as on an SUV).
for tag, x in (("F", FA), ("R", RA)):
    cut(["CYLINDER %.0f %.0f @0,0,0" % (TYRE_R + 40, W + 200)], "Arch" + tag, ["ROTATE Arch%s 90,0,0" % tag, "MOVE %.1f,%.1f,%.1f Arch%s" % (x, (W + 200) / 2, TYRE_R, tag)])
# Grille recess in the nose, and a dark insert in it.
cut(["BOX 140 1100 250 @0,0,0"], "GrilleCut", ["MOVE -40,0,620 GrilleCut"])
out += ["BOX 70 1090 240 @0,0,0", "RENAME Grille", "MOVE 15,0,625 Grille"]
# Clamshell bonnet: its shut lines run along the fender tops; door shut lines and sills.
for sgn in (1, -1):
    cut(["BOX 1380 3 160 @0,0,0"], "Gap", ["MOVE %.1f,%.1f,%.1f Gap" % (820, sgn * (interp(half, 900) - 80), interp(top, 900) - 110)])
    for x0 in (1560.0, 2700.0, 3620.0):
        hw = interp(half, x0); zb = interp(bot, x0); zt = interp(top, x0)
        cut(["BOX 3 150 %.0f @0,0,0" % (zt - zb - 150)], "Gap", ["MOVE %.1f,%.1f,%.1f Gap" % (x0, sgn * (hw - 30), zb + 105)])
    hw = interp(half, 2600); zb = interp(bot, 2600)
    cut(["BOX 2060 150 3 @0,0,0"], "Gap", ["MOVE %.1f,%.1f,%.1f Gap" % (2590, sgn * (hw - 30), zb + 105)])
# Lamps, side vents, mirrors.
for sgn, side in ((1, "L"), (-1, "R")):
    out += ["ELLIPSOID 110 210 32 @0,0,0", "RENAME Head%s" % side, "ROTATE Head%s 0,0,%d" % (side, -sgn * 18),
            "MOVE %.1f,%.1f,%.1f Head%s" % (70, sgn * 720, 905, side)]
    out += ["ELLIPSOID 50 120 14 @0,0,0", "RENAME Lamp%s" % side, "ROTATE Lamp%s 0,0,%d" % (side, -sgn * 18),
            "MOVE %.1f,%.1f,%.1f Lamp%s" % (45, sgn * 745, 925, side)]
    out += ["ELLIPSOID 40 250 40 @0,0,0", "RENAME Tail%s" % side, "MOVE %.1f,%.1f,%.1f Tail%s" % (5015, sgn * 760, 1010, side)]
    out += ["BOX 260 30 30 @0,0,0", "RENAME Vent%s" % side, "MOVE %.1f,%.1f,%.1f Vent%s" % (1720, sgn * (interp(half, 1720) - 8), 880, side)]
    hwm = interp(half, 1650)
    out += ["ELLIPSOID 95 120 70 @0,0,0", "RENAME Mirror%s" % side, "MOVE %.1f,%.1f,%.1f Mirror%s" % (1650, sgn * (hwm + 90), interp(top, 1650) + 40, side),
            "BOX 80 140 30 @0,0,0", "RENAME Arm%s" % side, "MOVE %.1f,%.1f,%.1f Arm%s" % (1650, sgn * (hwm - 5), interp(top, 1650) + 60, side),
            "BOOLEAN Mirror%s UNION Arm%s" % (side, side)]
# A dark band across the tailgate between the tail lamps.
out += ["BOX 30 1300 60 @0,0,0", "RENAME Band", "MOVE 5030,0,995 Band"]
for tag, x in (("FL", FA), ("FR", FA), ("RL", RA), ("RR", RA)):
    sgn = 1 if tag[1] == "L" else -1
    out += wheel(tag, x, sgn * (interp(half, x) - TYRE_W / 2 - 30), TYRE_R, sgn)
paint = "#5f6569"  # a dark metallic grey
out += ["MATERIAL Body MirrorL MirrorR COLOR %s" % paint, "MATERIAL Body MirrorL MirrorR FINISH polished",
        "MATERIAL Glasshouse glass", "MATERIAL Glasshouse COLOR #08090b", "MATERIAL Glasshouse TRANSMISSION 0.35", "MATERIAL Glasshouse ROUGHNESS 0.02",
        "MATERIAL HeadL HeadR glass", "MATERIAL HeadL HeadR COLOR #16191d", "MATERIAL HeadL HeadR TRANSMISSION 0.3",
        "MATERIAL LampL LampR chrome", "MATERIAL LampL LampR EMISSION 0.5",
        "MATERIAL TailL TailR COLOR #b8101a", "MATERIAL TailL TailR EMISSION 0.6", "MATERIAL TailL TailR ROUGHNESS 0.1",
        "MATERIAL Grille Band VentL VentR COLOR #0b0c0e", "MATERIAL Grille Band VentL VentR ROUGHNESS 0.35"]
names = ["Body", "Glasshouse", "Grille", "Band"] + [p + t for p in ("Head", "Lamp", "Tail", "Vent", "Mirror") for t in ("L", "R")] + \
        [p + t for p in ("Tyre", "Rim", "Disc", "Caliper") for t in ("FL", "FR", "RL", "RR")]
out += ["MIRROR %s PLANE YZ AT %.0f,0,0 MOVE" % (n, L / 2) for n in names]
print("\n".join(out))

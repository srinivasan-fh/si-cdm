#!/usr/bin/env python3
"""Writes docs/examples/sportscar.txt: a rear-engined sports coupe exterior as SI-CDM commands.

    tools/sportscar.py > docs/examples/sportscar.txt

Sizes (mm): 4519 long, 1852 wide, 1298 high, 2450 wheelbase, 110 ground clearance; staggered
wheels, 235/40 R20 front (696 across) and 305/30 R21 rear (716). Built nose at x = 0 (Y across,
Z up) from the tables below, then mirrored so the nose points +X. Raised front fenders with
upright oval headlamps and a low bonnet between them; a teardrop glasshouse whose roof runs in
one slope down to the engine lid; wide rear haunches; a full-width rear light bar.
"""
import math

L, W, H, WB = 4519.0, 1852.0, 1298.0, 2450.0
FA = 960.0; RA = FA + WB
FRONT = dict(R=348.0, W=235.0, rim=254.0)   # 235/40 R20
REAR = dict(R=358.0, W=305.0, rim=267.0)    # 305/30 R21

def interp(table, x):
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x0 <= x <= x1:
            t = (x - x0) / (x1 - x0); t = t * t * (3 - 2 * t)
            return y0 + (y1 - y0) * t
    return table[-1][1] if x > table[-1][0] else table[0][1]

# Lines along x: fender / belt / deck top, bonnet dip between the fenders, underbody, half width, roof.
top  = [(0, 560), (100, 640), (450, 720), (960, 780), (1350, 790), (1900, 810), (2700, 840), (3410, 870), (3900, 860), (4300, 830), (4519, 770)]
dip  = [(0, 30), (300, 70), (800, 85), (1250, 60), (1500, 0), (4519, 0)]
bot  = [(0, 308), (150, 208), (500, 148), (960, 130), (2400, 120), (3410, 130), (4100, 178), (4519, 338)]
half = [(0, 645), (120, 795), (450, 885), (960, 910), (1700, 895), (2500, 900), (3410, 931), (4000, 917), (4380, 855), (4519, 765)]
G0, G1 = 1480.0, 4150.0
roof = [(G0, 770), (1800, 1070), (2150, 1265), (2450, 1295), (2850, 1267), (3350, 1150), (3800, 990), (G1, 865)]

def section(x, k=1.0):
    hw, zb, zt = interp(half, x), interp(bot, x), interp(top, x)
    if k != 1.0:
        zc = 0.5 * (zb + zt); hw *= k; zb = zc - (zc - zb) * k; zt = zc + (zt - zc) * k
    h = zt - zb; d = interp(dip, x) * k
    pts = [(0, zb), (0.72 * hw, zb), (0.92 * hw, zb + 0.08 * h), (0.985 * hw, zb + 0.26 * h), (hw, zb + 0.52 * h),
           (0.975 * hw, zb + 0.78 * h), (0.90 * hw, zt - 0.04 * h), (0.74 * hw, zt), (0.50 * hw, zt - 0.75 * d), (0.22 * hw, zt - d), (0, zt - d)]
    return pts + [(-y, z) for (y, z) in reversed(pts[1:-1])]

def gsection(x):
    zr = interp(roof, x); base = interp(top, x) - 22 - interp(dip, x)
    taper = min(1.0, (G1 - x) / 500.0, (x - G0) / 260.0 + 0.55)
    hb = (interp(half, x) - 195) * (0.70 + 0.30 * max(taper, 0.0)); hr = hb * 0.66
    h = max(zr - base, 30.0)
    pts = [(0, base), (hb, base), (hb * 0.96, base + 0.30 * h), (hr * 1.08, base + 0.82 * h), (hr * 0.62, base + h * 0.985), (0, base + h)]
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

def wheel(tag, xc, yc, outboard, R, W, rim):
    s = 1 if outboard > 0 else -1
    k = rim / 228.0; w = W / 235.0
    out = ["PLANE RIGHT", pline(tyre_section(-W / 2, W / 2, rim - 4, R, 40, 10)), "REVOLVE AXIS -400,0 400,0 ANGLE 360", "RENAME Tyre%s" % tag]
    prof = [(-112 * w, rim - 16), (-112 * w, rim), (112 * w, rim), (120 * w, rim - 8), (120 * w, 48 * k), (96 * w, 48 * k), (96 * w, rim - 26)]
    out += [pline([(y * s, z) for (y, z) in prof]), "REVOLVE AXIS -400,0 400,0 ANGLE 360", "RENAME Rim%s" % tag]
    # Five big windows: a five-spoke wheel.
    out += ["CYLINDER %.0f 500 @0,0,0" % (60 * k), "RENAME Win%s" % tag, "ROTATE Win%s 90,0,0" % tag,
            "MOVE 0,250,%.1f Win%s" % (138 * k, tag), 'PATTERN Win%s CIRCULAR 5 AXIS Y AT 0,0,0 SUBTRACT "Rim%s"' % (tag, tag)]
    out += ["CYLINDER %.0f 32 @0,0,0" % (178 * k), "RENAME Disc%s" % tag, "ROTATE Disc%s 90,0,0" % tag, "MOVE 0,%.1f,0 Disc%s" % (s * 80 * w + 16, tag)]
    out += ["BOX 90 40 150 @0,0,0", "RENAME Caliper%s" % tag, "MOVE %.1f,%.1f,%.1f Caliper%s" % (-185 if xc < 2000 else 185, s * 66 * w, -75, tag)]
    for p in ("Tyre", "Rim", "Disc", "Caliper"): out.append("MOVE %.1f,%.1f,%.1f %s%s" % (xc, yc, R, p, tag))
    out += ["MATERIAL Tyre%s rubber" % tag, "MATERIAL Rim%s aluminium" % tag, "MATERIAL Rim%s COLOR #c5c8cd" % tag, "MATERIAL Rim%s FINISH satin" % tag,
            "MATERIAL Disc%s steel" % tag, "MATERIAL Disc%s COLOR #5a5d62" % tag, "MATERIAL Caliper%s COLOR #d8b400" % tag]
    return out

out = ["NEW", "UNITS mm", "PLANE RIGHT"]
stations = [0, 45, 150, 450, 960, 1500, 2100, 2700, 3410, 3900, 4250, 4440, 4519]
for x in stations:
    out.append(spline(section(45 if x == 0 else 4440 if x == L else x, 0.6 if x == 0 else 0.72 if x == L else 1.0)))
out.append("LOFT " + " ".join('"Spline %d" AT %.0f' % (i + 1, x) for i, x in enumerate(stations)) + " SMOOTH")
out.append('RENAME "Loft 1" Body')
gst = [G0 + 1, 1650, 1800, 2150, 2450, 2850, 3350, 3800, G1 - 1]
k0 = len(stations)
for x in gst: out.append(spline(gsection(x)))
out.append("LOFT " + " ".join('"Spline %d" AT %.0f' % (k0 + i + 1, x) for i, x in enumerate(gst)) + " SMOOTH")
out.append('RENAME "Loft 2" Glasshouse')

def cut(make, tool, place):
    out.extend(make); out.append("RENAME %s" % tool); out.extend(place); out.append("BOOLEAN Body SUBTRACT %s" % tool)
# Wheel arches, cut from each side only (a cylinder right across would cut the bonnet, which sits
# between the fenders lower than the arches' tops).
for tag, x, wh in (("F", FA, FRONT), ("R", RA, REAR)):
    for sgn in (1, -1):
        hw = interp(half, x); depth = wh["W"] + 120
        cut(["CYLINDER %.0f %.0f @0,0,0" % (wh["R"] + 32, depth)], "Arch" + tag,
            ["ROTATE Arch%s 90,0,0" % tag, "MOVE %.1f,%.1f,%.1f Arch%s" % (x, sgn * (hw + 80) if sgn > 0 else sgn * (hw + 80) + depth, wh["R"], tag)])
# Front intakes: a wide centre one and two at the corners.
cut(["BOX 120 640 110 @0,0,0"], "Intake", ["MOVE -30,0,250 Intake"])
for sgn in (1, -1):
    cut(["BOX 120 240 120 @0,0,0"], "Intake", ["MOVE -30,%.1f,245 Intake" % (sgn * 560)])
# Door shut lines and the front lid's edges (between the fenders).
for sgn in (1, -1):
    cut(["BOX 1180 3 140 @0,0,0"], "Gap", ["MOVE %.1f,%.1f,%.1f Gap" % (780, sgn * 0.62 * interp(half, 800), interp(top, 800) - interp(dip, 800) - 60)])
    for x0 in (1540.0, 2780.0):
        hw = interp(half, x0); zb = interp(bot, x0); zt = interp(top, x0)
        cut(["BOX 3 150 %.0f @0,0,0" % (zt - zb - 120), ], "Gap", ["MOVE %.1f,%.1f,%.1f Gap" % (x0, sgn * (hw - 30), zb + 90)])
# Headlamps standing up on the front fenders; a full-width rear light bar; an engine grille; mirrors.
for sgn, side in ((1, "L"), (-1, "R")):
    x = 260.0
    out += ["ELLIPSOID 120 110 125 @0,0,0", "RENAME Head%s" % side, "ROTATE Head%s 0,-38,0" % side,
            "MOVE %.1f,%.1f,%.1f Head%s" % (x, sgn * 0.70 * interp(half, x), interp(top, x) - 205, side)]
    out += ["ELLIPSOID 40 70 72 @0,0,0", "RENAME Lamp%s" % side, "ROTATE Lamp%s 0,-38,0" % side,
            "MOVE %.1f,%.1f,%.1f Lamp%s" % (x - 70, sgn * 0.70 * interp(half, x), interp(top, x) - 135, side)]
    hwm = interp(half, 1620)
    out += ["ELLIPSOID 80 95 48 @0,0,0", "RENAME Mirror%s" % side, "MOVE %.1f,%.1f,%.1f Mirror%s" % (1620, sgn * (hwm + 45), interp(top, 1620) + 30, side),
            "BOX 70 110 22 @0,0,0", "RENAME Arm%s" % side, "MOVE %.1f,%.1f,%.1f Arm%s" % (1620, sgn * (hwm - 30), interp(top, 1620) + 40, side),
            "BOOLEAN Mirror%s UNION Arm%s" % (side, side)]
out += ["ELLIPSOID 30 760 16 @0,0,0", "RENAME Taillight", "MOVE %.1f,0,%.1f Taillight" % (4492, interp(top, 4492) - 60)]
out += ["BOX 420 760 10 @0,0,0", "RENAME EngineGrille", "MOVE %.1f,0,%.1f EngineGrille" % (4160, interp(top, 4160) - 7)]
out += ["BOX 60 1100 90 @0,0,0", "RENAME Diffuser", "MOVE %.1f,0,%.1f Diffuser" % (4470, interp(bot, 4470) + 5)]
for tag, x, wh in (("FL", FA, FRONT), ("FR", FA, FRONT), ("RL", RA, REAR), ("RR", RA, REAR)):
    sgn = 1 if tag[1] == "L" else -1
    out += wheel(tag, x, sgn * (interp(half, x) - wh["W"] / 2 - 28), sgn, wh["R"], wh["W"], wh["rim"])
paint = "#b3121b"
out += ["MATERIAL Body MirrorL MirrorR COLOR %s" % paint, "MATERIAL Body MirrorL MirrorR FINISH polished",
        "MATERIAL Glasshouse glass", "MATERIAL Glasshouse COLOR #08090b", "MATERIAL Glasshouse TRANSMISSION 0.4", "MATERIAL Glasshouse ROUGHNESS 0.02",
        "MATERIAL HeadL HeadR glass", "MATERIAL HeadL HeadR COLOR #1b1e22", "MATERIAL HeadL HeadR TRANSMISSION 0.3", "MATERIAL HeadL HeadR ROUGHNESS 0.02",
        "MATERIAL LampL LampR chrome", "MATERIAL LampL LampR EMISSION 0.6",
        "MATERIAL Taillight COLOR #c0101a", "MATERIAL Taillight EMISSION 0.7", "MATERIAL Taillight ROUGHNESS 0.08",
        "MATERIAL EngineGrille Diffuser COLOR #0b0c0e", "MATERIAL EngineGrille Diffuser ROUGHNESS 0.4"]
names = ["Body", "Glasshouse", "Taillight", "EngineGrille", "Diffuser"] + [p + t for p in ("Head", "Lamp", "Mirror") for t in ("L", "R")] + \
        [p + t for p in ("Tyre", "Rim", "Disc", "Caliper") for t in ("FL", "FR", "RL", "RR")]
out += ["MIRROR %s PLANE YZ AT %.1f,0,0 MOVE" % (n, L / 2) for n in names]
print("\n".join(out))

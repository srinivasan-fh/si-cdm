#!/usr/bin/env python3
"""Writes docs/examples/ev-sedan.txt: a fastback EV sedan exterior as SI-CDM commands.

    tools/ev_sedan.py > docs/examples/ev-sedan.txt

Every size comes from the tables below (mm; built with the nose at x = 0, Y across, Z up, then
mirrored so the nose points +X): the bonnet / belt / deck line, the bonnet's dip below the
fender crowns, the underbody, the half width (waist and rear haunches), the roof line. The
body is a solid lofted through closed sections; the glasshouse a second loft; wheels are
revolved tyre and rim profiles with spoke windows cut in a circular pattern.
"""
import math

def rounded_rect(y0, y1, z0, z1, r_out, r_in, n=8):
    # Tyre section in (y, z): outer (tread) corners round r_out, bead corners r_in.
    pts = []
    def arc(cy, cz, r, a0, a1):
        for i in range(n + 1):
            a = a0 + (a1 - a0) * i / n
            pts.append((cy + r * math.cos(a), cz + r * math.sin(a)))
    arc(y1 - r_out, z1 - r_out, r_out, 0, math.pi / 2)            # outboard tread shoulder
    arc(y0 + r_out, z1 - r_out, r_out, math.pi / 2, math.pi)      # inboard tread shoulder
    arc(y0 + r_in, z0 + r_in, r_in, math.pi, 1.5 * math.pi)       # inboard bead
    arc(y1 - r_in, z0 + r_in, r_in, 1.5 * math.pi, 2 * math.pi)   # outboard bead
    return pts
def pline(pts):
    return "PLINE " + " ".join("%.2f,%.2f" % p for p in pts) + " CLOSE"
def wheel(tag, xc, yc, zc, outboard, R=335.0, W=235.0, rim=228.0):
    s = +1 if outboard > 0 else -1
    out = ["PLANE RIGHT"]
    # Tyre: revolved about the axis (along Y through the origin), then moved to the hub.
    out.append(pline(rounded_rect(-W / 2, W / 2, rim - 4, R, 55, 12)))
    out.append("REVOLVE AXIS -300,0 300,0 ANGLE 360"); out.append("RENAME Tyre%s" % tag)
    # Rim: barrel, lip and a dished face on the outboard side.
    prof = [(-112, rim - 16), (-112, rim), (112, rim), (120, rim - 8), (120, 48), (96, 48), (96, rim - 26)]
    prof = [(y * s, z) for (y, z) in prof]
    out.append(pline(prof)); out.append("REVOLVE AXIS -300,0 300,0 ANGLE 360"); out.append("RENAME Rim%s" % tag)
    # Five spoke windows cut through the face.
    out.append("CYLINDER 66 400 @0,0,0"); out.append("RENAME Win%s" % tag)
    out.append("ROTATE Win%s 90,0,0" % tag)
    out.append("MOVE 0,200,138 Win%s" % tag)  # straight through the face (y -200..200)
    out.append('PATTERN Win%s CIRCULAR 5 AXIS Y AT 0,0,0 SUBTRACT "Rim%s"' % (tag, tag))
    # Brake disc behind the face, and a caliper at the back.
    out.append("CYLINDER 172 26 @0,0,0"); out.append("RENAME Disc%s" % tag)
    out.append("ROTATE Disc%s 90,0,0" % tag); out.append("MOVE 0,%.1f,0 Disc%s" % (s * 75 + 13, tag))
    out.append("BOX 70 34 120 @0,0,0"); out.append("RENAME Caliper%s" % tag)
    out.append("MOVE %.1f,%.1f,%.1f Caliper%s" % (-150 if xc < 2000 else 150, s * 62, -60, tag))
    parts = ["Tyre", "Rim", "Disc", "Caliper"]
    for p in parts: out.append("MOVE %.1f,%.1f,%.1f %s%s" % (xc, yc, zc, p, tag))
    out += ["MATERIAL Tyre%s rubber" % tag, "MATERIAL Rim%s aluminium" % tag, "MATERIAL Rim%s FINISH satin" % tag,
            "MATERIAL Disc%s steel" % tag, "MATERIAL Disc%s COLOR #4a4d52" % tag, "MATERIAL Disc%s FINISH brushed" % tag, "MATERIAL Caliper%s COLOR #202226" % tag]
    return out
L, W, H, WB = 4720.0, 1850.0, 1440.0, 2875.0
FA = 840.0; RA = FA + WB
TYRE_R = 335.0
def interp(table, x):
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x0 <= x <= x1:
            t = (x - x0) / (x1 - x0); t = t * t * (3 - 2 * t)
            return y0 + (y1 - y0) * t
    return table[-1][1] if x > table[-1][0] else table[0][1]
# Lines of the body, along x.
top  = [(0, 600), (120, 690), (450, 790), (840, 865), (1350, 935), (2500, 960), (3715, 985), (4300, 1000), (4550, 1000), (4720, 940)]  # fender / belt / deck top
dip  = [(0, 10), (300, 34), (840, 36), (1250, 18), (1450, 0), (4720, 0)]       # bonnet centre below the fender crowns
bot  = [(0, 354), (150, 244), (450, 178), (840, 158), (2400, 148), (4000, 168), (4500, 268), (4720, 428)]
half = [(0, 568), (120, 768), (450, 885), (840, 921), (1500, 915), (2400, 908), (3300, 917), (3715, 925), (4300, 895), (4600, 805), (4720, 698)]  # waist, rear haunch
G0, G1 = 1300.0, 4150.0
roof = [(G0, 940), (1700, 1180), (2150, 1400), (2600, 1438), (3100, 1390), (3700, 1170), (G1, 1000)]
def section(x, k=1.0):
    hw, zb, zt = interp(half, x), interp(bot, x), interp(top, x)
    if k != 1.0:   # an end: drawn in about its middle
        zc = 0.5 * (zb + zt); hw *= k; zb = zc - (zc - zb) * k; zt = zc + (zt - zc) * k
    h = zt - zb
    d = interp(dip, x) * k
    zs = zb + 0.70 * h
    pts = [(0, zb), (0.70 * hw, zb), (0.92 * hw, zb + 0.06 * h), (0.985 * hw, zb + 0.22 * h), (hw, zb + 0.47 * h),
           (0.997 * hw, zs - 0.05 * h), (0.982 * hw, zs), (0.94 * hw, zt - 0.07 * h), (0.74 * hw, zt), (0.36 * hw, zt - 0.8 * d), (0, zt - d)]
    return pts + [(-y, z) for (y, z) in reversed(pts[1:-1])]
def gsection(x):
    zr = interp(roof, x); base = interp(top, x) - 25 - interp(dip, x)
    taper = min(1.0, (G1 - x) / 450.0, (x - G0) / 250.0 + 0.6)
    hb = (interp(half, x) - 95) * (0.80 + 0.20 * max(taper, 0.0)); hr = hb * 0.64
    h = max(zr - base, 40.0)
    pts = [(0, base), (hb, base), (hb * 0.97, base + 0.30 * h), (hr * 1.07, base + 0.84 * h), (hr * 0.6, base + h * 0.985), (0, base + h)]
    return pts + [(-y, z) for (y, z) in reversed(pts[1:-1])]
def spline(pts): return "SPLINE " + " ".join("%.1f,%.1f" % p for p in pts) + " CLOSE"
out = ["NEW", "UNITS mm", "PLANE RIGHT"]
stations = [0, 60, 160, 450, 840, 1350, 1900, 2500, 3100, 3715, 4300, 4560, 4660, 4720]
for x in stations:
    out.append(spline(section(60 if x == 0 else 4660 if x == 4720 else x, 0.55 if x == 0 else 0.62 if x == 4720 else 1.0)))
out.append("LOFT " + " ".join('"Spline %d" AT %.0f' % (i + 1, x) for i, x in enumerate(stations)) + " SMOOTH")
out.append('RENAME "Loft 1" Body')
gst = [G0 + 1, 1500, 1700, 2150, 2600, 3100, 3700, 4000, G1 - 1]
k0 = len(stations)
for x in gst: out.append(spline(gsection(x)))
out.append("LOFT " + " ".join('"Spline %d" AT %.0f' % (k0 + i + 1, x) for i, x in enumerate(gst)) + " SMOOTH")
out.append('RENAME "Loft 2" Glasshouse')
def cut(cmds, name, tool):  # a tool body made by cmds, renamed, subtracted from `name`
    out.extend(cmds); out.append("RENAME %s" % tool); out.append("BOOLEAN %s SUBTRACT %s" % (name, tool))
# Wheel arches.
for tag, x in (("F", FA), ("R", RA)):
    cut(["CYLINDER %.0f %.0f @0,0,0" % (TYRE_R + 42, W + 200)], "Body", "Arch" + tag)
    out[-2:-2] = []  # (placement before the cut)
    out.insert(len(out) - 1, "ROTATE Arch%s 90,0,0" % tag)
    out.insert(len(out) - 1, "MOVE %.1f,%.1f,%.1f Arch%s" % (x, (W + 200) / 2, TYRE_R, tag))
# Lower front intake.
out += ["BOX 120 1000 110 @0,0,0", "RENAME Intake", "MOVE -20,0,330 Intake", "BOOLEAN Body SUBTRACT Intake"]
# Door shut lines on both sides: two verticals and the sill line, cut 3 mm wide into the skin.
for sgn in (1, -1):
    for x0 in (1440.0, 2480.0, 3420.0):
        hw = interp(half, x0); zb = interp(bot, x0); zt = interp(top, x0)
        out += ["BOX 3 150 %.0f @0,0,0" % (zt - zb - 160), "RENAME Gap", "MOVE %.1f,%.1f,%.1f Gap" % (x0, sgn * (hw - 30), zb + 110), "BOOLEAN Body SUBTRACT Gap"]
    hw = interp(half, 2400); zb = interp(bot, 2400)
    out += ["BOX 1980 150 3 @0,0,0", "RENAME Gap", "MOVE %.1f,%.1f,%.1f Gap" % (2430, sgn * (hw - 30), zb + 110), "BOOLEAN Body SUBTRACT Gap"]
# Lamps, a dark intake insert, mirrors.
for sgn, side in ((1, "L"), (-1, "R")):
    out += ["ELLIPSOID 300 105 44 @0,0,0", "RENAME Head%s" % side, "ROTATE Head%s 0,%d,%d" % (side, 12, -sgn * 30),
            "MOVE %.1f,%.1f,%.1f Head%s" % (380, sgn * 650, interp(top, 380) - 118, side)]
    out += ["ELLIPSOID 120 45 20 @0,0,0", "RENAME Lamp%s" % side, "ROTATE Lamp%s 0,%d,%d" % (side, 12, -sgn * 30),
            "MOVE %.1f,%.1f,%.1f Lamp%s" % (320, sgn * 705, interp(top, 320) - 100, side)]
    out += ["ELLIPSOID 40 330 30 @0,0,0", "RENAME Tail%s" % side, "ROTATE Tail%s 0,0,%d" % (side, sgn * 12),
            "MOVE %.1f,%.1f,%.1f Tail%s" % (4655, sgn * 520, interp(top, 4655) - 95, side)]
    hwm = interp(half, 1560)
    out += ["ELLIPSOID 85 105 55 @0,0,0", "RENAME Mirror%s" % side, "MOVE %.1f,%.1f,%.1f Mirror%s" % (1560, sgn * (hwm + 70), interp(top, 1560) + 20, side)]
    out += ["BOX 70 120 26 @0,0,0", "RENAME Arm%s" % side, "MOVE %.1f,%.1f,%.1f Arm%s" % (1560, sgn * (hwm - 10), interp(top, 1560) + 40, side),
            "BOOLEAN Mirror%s UNION Arm%s" % (side, side)]
out += ["BOX 60 960 70 @0,0,0", "RENAME Insert", "MOVE 22,0,350 Insert"]
for tag, x in (("FL", FA), ("FR", FA), ("RL", RA), ("RR", RA)):
    sgn = 1 if tag[1] == "L" else -1
    out += wheel(tag, x, sgn * (interp(half, x) - 117 - 22), TYRE_R, sgn)
out += ["MATERIAL Body MirrorL MirrorR COLOR #a50d1b", "MATERIAL Body MirrorL MirrorR FINISH polished",
        "MATERIAL Glasshouse glass", "MATERIAL Glasshouse COLOR #0b0e12", "MATERIAL Glasshouse TRANSMISSION 0.5", "MATERIAL Glasshouse ROUGHNESS 0.02",
        "MATERIAL HeadL HeadR glass", "MATERIAL HeadL HeadR COLOR #14171b", "MATERIAL HeadL HeadR TRANSMISSION 0.3", "MATERIAL HeadL HeadR ROUGHNESS 0.03",
        "MATERIAL LampL LampR chrome", "MATERIAL LampL LampR EMISSION 0.4",
        "MATERIAL TailL TailR COLOR #c4101c", "MATERIAL TailL TailR EMISSION 0.6", "MATERIAL TailL TailR ROUGHNESS 0.1",
        "MATERIAL Insert COLOR #0a0b0d", "MATERIAL Insert ROUGHNESS 0.6"]
names = ["Body", "Glasshouse", "Insert"] + [p + t for p in ("Head", "Lamp", "Tail", "Mirror") for t in ("L", "R")] + \
        [p + t for p in ("Tyre", "Rim", "Disc", "Caliper") for t in ("FL", "FR", "RL", "RR")]
out += ["MIRROR %s PLANE YZ AT %.0f,0,0 MOVE" % (n, L / 2) for n in names]
print("\n".join(out))

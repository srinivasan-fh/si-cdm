"""Shared parts of the car generators (tools/cars.py): SI-CDM commands for a car's exterior.

A car is drawn nose at x = 0, Y across and Z up, from tables of its lines along x:

    top   the top of the body: bonnet, belt line, deck
    dip   how far the bonnet sits below the fenders' crowns between them
    bot   the underbody (ground clearance between the axles)
    half  half the body's width
    roof  the glasshouse's top, from G0 (the windscreen's foot) to G1 (the rear glass's foot)

Closed cross-sections through those lines (on PLANE RIGHT: u across, v up) are lofted into a
solid body and a second loft makes the glasshouse. Smooth sections are splines; a car with
creases uses polylines, which loft corner to corner. Arches, intakes and shut lines are cut
in, wheels are revolved from their tyre size, and at the end every part is mirrored about
x = L/2 so the nose points +X, as the render's iso camera expects.
"""
import math


def tyre(code):
    """A tyre size code, "245/30 R20", as its outer radius R, width W and rim radius (mm)."""
    w, rest = code.replace(" ", "").split("/")
    aspect, rim_in = rest.split("R")
    w, aspect, rim_in = float(w), float(aspect), float(rim_in)
    rim = rim_in * 25.4 / 2
    return dict(code=code, W=w, rim=rim, R=rim + w * aspect / 100.0)


def interp(table, x):
    """The table's value at x, eased between its points (no overshoot)."""
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x0 <= x <= x1:
            t = (x - x0) / (x1 - x0)
            t = t * t * (3 - 2 * t)
            return y0 + (y1 - y0) * t
    return table[-1][1] if x > table[-1][0] else table[0][1]


def spline(pts):
    return "SPLINE " + " ".join("%.1f,%.1f" % p for p in pts) + " CLOSE"


def pline(pts):
    return "PLINE " + " ".join("%.2f,%.2f" % p for p in pts) + " CLOSE"


def mirrored(pts):
    """A half section (centre bottom to centre top, +y side) closed with its other half."""
    return pts + [(-y, z) for (y, z) in reversed(pts[1:-1])]


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


class Car:
    """Collects a car's commands. Lines are tables of (x, value); see the module's notes."""

    def __init__(self, L, WB, FA, front, rear, top, dip, bot, half, roof, G0, G1, tune=(1.0, 0.0, 0.0)):
        """tune: (width factor, roof lift, underbody lift) making up for the splines through
        the sections bulging a little past the lines' points, so the overall sizes come out
        exactly (measured on the built car and set once)."""
        sw, dh, dc = tune
        half = [(x, v * sw) for x, v in half]
        roof = [roof[0]] + [(x, v + dh) for x, v in roof[1:-1]] + [roof[-1]]  # (its ends stay on the belt)
        bot = [(x, v + dc) for x, v in bot]
        self.L, self.WB, self.FA, self.RA = L, WB, FA, FA + WB
        self.front, self.rear = tyre(front), tyre(rear)
        self.top, self.dip, self.bot, self.half, self.roof = top, dip, bot, half, roof
        self.G0, self.G1 = G0, G1
        self.out = ["NEW", "UNITS mm", "PLANE RIGHT"]
        self.curves = 0      # sketch curves made so far (they are numbered by kind)
        self.splines = 0
        self.plines = 0
        self.lofts = 0
        self.parts = []      # every body, mirrored at the end
        self.paint = ["Body"]

    # Lines at x.
    def hw(self, x): return interp(self.half, x)
    def zt(self, x): return interp(self.top, x)
    def zb(self, x): return interp(self.bot, x)
    def zd(self, x): return interp(self.dip, x)
    def zr(self, x): return interp(self.roof, x)

    def _curve(self, pts, crease):
        if crease:
            self.plines += 1
            self.out.append(pline(pts))
            return '"Polyline %d"' % self.plines
        self.splines += 1
        self.out.append(spline(pts))
        return '"Spline %d"' % self.splines

    def _loft(self, names, xs, name):
        self.lofts += 1
        self.out.append("LOFT " + " ".join("%s AT %.0f" % (n, x) for n, x in zip(names, xs)) + " SMOOTH")
        self.out.append('RENAME "Loft %d" %s' % (self.lofts, name))
        self.parts.append(name)

    def section(self, x, shape, k=1.0):
        """The body's section at x. `shape` runs from the centre of the underbody to the
        centre of the bonnet / roof: (across as a share of the half width, up as a share of
        the body's height, how much of the bonnet's dip it takes). k shrinks it about its
        middle (the nose and tail, drawn small so the ends come out round)."""
        hw, zb, zt = self.hw(x), self.zb(x), self.zt(x)
        if k != 1.0:
            zc = 0.5 * (zb + zt)
            hw *= k
            zb = zc - (zc - zb) * k
            zt = zc + (zt - zc) * k
        h, d = zt - zb, self.zd(x) * k
        return mirrored([(yf * hw, zb + zf * h - df * d) for (yf, zf, df) in shape])

    def body(self, stations, shape, nose=(45, 0.6), tail=None, crease=False):
        """The body lofted through sections at these stations; the end sections are drawn
        at nose[0] / tail[0] in from the ends, shrunk by nose[1] / tail[1]."""
        tail = tail or (self.L - 80, 0.72)
        names = []
        for x in stations:
            if x == 0: xs, k = nose
            elif x == self.L: xs, k = tail
            else: xs, k = x, 1.0
            names.append(self._curve(self.section(xs, shape, k), crease))
        self._loft(names, stations, "Body")

    def glasshouse(self, stations, shape, inset, sink=22, taper=(260, 0.55, 500, 0.7), crease=False, name="Glasshouse"):
        """The glasshouse lofted from G0 to G1 on the belt line, set in from the body's side by
        `inset`. `shape` runs from the foot of its side to the centre of the roof: (across as
        a share of its half width at the foot, up as a share of its height). taper: (length
        over which the front narrows, its least width there, the same at the rear...) as
        (front length, front start share, rear length, least width)."""
        fl, f0, rl, least = taper
        names = []
        for x in stations:
            zr = self.zr(x)
            base = self.zt(x) - sink - self.zd(x)
            t = min(1.0, (self.G1 - x) / rl, (x - self.G0) / fl + f0)
            hb = (self.hw(x) - inset) * (least + (1 - least) * max(t, 0.0))
            h = max(zr - base, 30.0)
            # (The centre of the foot sits a little low: a straight run through it would be
            # dropped as no corner, and polyline sections must keep the same corners.)
            pts = [(0, base - 2)] + [(yf * hb, base + zf * h) for (yf, zf) in shape] + [(0, base + h)]
            names.append(self._curve(mirrored(pts), crease))
        self._loft(names, stations, name)

    # Bodies and cuts.
    def add(self, name, make, *place, mirror=True):
        """A part: the command that makes it, renamed, then moved / rotated into place."""
        self.out.append(make)
        self.out.append("RENAME %s" % name)
        self.out.extend(c.replace("$", name) for c in place)
        if mirror: self.parts.append(name)

    def cut(self, make, *place, tool="Cut", target="Body"):
        self.out.append(make)
        self.out.append("RENAME %s" % tool)
        self.out.extend(c.replace("$", tool) for c in place)
        self.out.append("BOOLEAN %s SUBTRACT %s" % (target, tool))

    def box_cut(self, sx, sy, sz, x, y, z):
        """A box cut centred across (y) and lengthwise (x); z is its bottom (BOX stands on z)."""
        self.cut("BOX %.0f %.0f %.0f @0,0,0" % (sx, sy, sz), "MOVE %.1f,%.1f,%.1f $" % (x, y, z))

    def arches(self, extra=32, side_depth=120, sides=0):
        """Wheel arches cut from each side only (a cylinder right across would cut a bonnet
        that sits lower between the fenders than the arches' tops). sides > 0: a prism of that
        many sides instead (6: a six-sided arch with a flat top), clear of the tyre by extra."""
        for x, t in ((self.FA, self.front), (self.RA, self.rear)):
            for sgn in (1, -1):
                depth = t["W"] + side_depth
                hw = self.hw(x)
                y = sgn * (hw + 80) if sgn > 0 else -(hw + 80) + depth
                if sides:
                    r = (t["R"] + extra) / math.cos(math.pi / sides)
                    make = "PRISM %.0f %.0f %d @0,0,0" % (r, depth, sides)
                else:
                    make = "CYLINDER %.0f %.0f @0,0,0" % (t["R"] + extra, depth)
                self.cut(make, "ROTATE $ 90,0,0", "MOVE %.1f,%.1f,%.1f $" % (x, y, t["R"]))

    def door_lines(self, xs, low=90, high=120, inset=30, sill=None):
        """Vertical shut lines at these x on both sides, from `low` above the underbody to
        `high` below the top; and a sill line along from sill[0] to sill[1]."""
        for sgn in (1, -1):
            for x in xs:
                zb, zt = self.zb(x), self.zt(x) - self.zd(x)
                self.box_cut(3, 150, zt - zb - low - high, x, sgn * (self.hw(x) - inset), zb + low)
            if sill:
                x0, x1 = sill
                xm = 0.5 * (x0 + x1)
                self.box_cut(x1 - x0, 150, 3, xm, sgn * (self.hw(xm) - inset), self.zb(xm) + low)

    def lid_lines(self, x0, x1, share, below=60):
        """A lid's side shut lines from x0 to x1, `share` of the half width out from the
        centre, `below` under the top at the middle."""
        xm = 0.5 * (x0 + x1)
        for sgn in (1, -1):
            self.box_cut(x1 - x0, 3, below + 80, xm, sgn * share * self.hw(xm), self.zt(xm) - self.zd(xm) - below)

    def cross_line(self, x, share=0.9, depth=140):
        """A shut line across the car at x (a bonnet's or boot lid's end)."""
        self.box_cut(3, 2 * share * self.hw(x), depth, x, 0, self.zt(x) - self.zd(x) - depth + 40)

    def mirrors(self, x, up=30, out=45, size=(80, 95, 48), share=1.0):
        """Door mirrors at x on arms, `out` beyond `share` of the half width, `up` above the top."""
        for sgn, side in ((1, "L"), (-1, "R")):
            hw = self.hw(x) * share
            m = "Mirror" + side
            self.add(m, "ELLIPSOID %d %d %d @0,0,0" % size, "MOVE %.1f,%.1f,%.1f $" % (x, sgn * (hw + out), self.zt(x) + up))
            self.add("Arm" + side, "BOX 70 %d 22 @0,0,0" % (out + 65), "MOVE %.1f,%.1f,%.1f $" % (x, sgn * (hw - 20), self.zt(x) + up + 10), mirror=False)
            self.out.append("BOOLEAN %s UNION Arm%s" % (m, side))
            self.paint.append(m)

    def wheels(self, spokes=5, win=60, ring=138, rim_color="#c5c8cd", caliper="#1c1d20", inset=28, shoulder=40, finish="satin"):
        """Four wheels at the axles: revolved tyres and rims, `spokes` windows of radius win at
        radius ring (both for a 228 mm rim, scaled to the rim), a disc and a caliper."""
        for tag in ("FL", "FR", "RL", "RR"):
            t = self.front if tag[0] == "F" else self.rear
            xc = self.FA if tag[0] == "F" else self.RA
            s = 1 if tag[1] == "L" else -1
            R, W, rim = t["R"], t["W"], t["rim"]
            k, w = rim / 228.0, W / 235.0
            yc = s * (self.hw(xc) - W / 2 - inset)
            o = self.out
            o += ["PLANE RIGHT", pline(tyre_section(-W / 2, W / 2, rim - 4, R, shoulder, 10)), "REVOLVE AXIS -400,0 400,0 ANGLE 360", "RENAME Tyre%s" % tag]
            prof = [(-112 * w, rim - 16), (-112 * w, rim), (112 * w, rim), (120 * w, rim - 8), (120 * w, 48 * k), (96 * w, 48 * k), (96 * w, rim - 26)]
            o += [pline([(y * s, z) for (y, z) in prof]), "REVOLVE AXIS -400,0 400,0 ANGLE 360", "RENAME Rim%s" % tag]
            self.plines += 2
            o += ["CYLINDER %.0f 500 @0,0,0" % (win * k), "RENAME Win%s" % tag, "ROTATE Win%s 90,0,0" % tag,
                  "MOVE 0,250,%.1f Win%s" % (ring * k, tag), 'PATTERN Win%s CIRCULAR %d AXIS Y AT 0,0,0 SUBTRACT "Rim%s"' % (tag, spokes, tag)]
            o += ["CYLINDER %.0f 30 @0,0,0" % (min(178 * k, rim - 50)), "RENAME Disc%s" % tag, "ROTATE Disc%s 90,0,0" % tag, "MOVE 0,%.1f,0 Disc%s" % (s * 80 * w + 16, tag)]
            o += ["BOX 90 40 150 @0,0,0", "RENAME Caliper%s" % tag,
                  "MOVE %.1f,%.1f,%.1f Caliper%s" % (-(rim - 70) if tag[0] == "F" else rim - 70, s * 66 * w, -75, tag)]
            for p in ("Tyre", "Rim", "Disc", "Caliper"): o.append("MOVE %.1f,%.1f,%.1f %s%s" % (xc, yc, R, p, tag))
            o += ["MATERIAL Tyre%s rubber" % tag, "MATERIAL Rim%s aluminium" % tag, "MATERIAL Rim%s COLOR %s" % (tag, rim_color), "MATERIAL Rim%s FINISH %s" % (tag, finish),
                  "MATERIAL Disc%s steel" % tag, "MATERIAL Disc%s COLOR #5a5d62" % tag, "MATERIAL Caliper%s COLOR %s" % (tag, caliper)]
            self.parts += [p + tag for p in ("Tyre", "Rim", "Disc", "Caliper")]

    def grille(self, w, h, z, depth=140, inset=10, name="Grille"):
        """A recess in the nose, w across and h high from z, with a dark insert in it (its face
        `inset` back from the nose)."""
        self.box_cut(depth, w, h, depth / 2 - 40, 0, z)
        self.add(name, "BOX %.0f %.0f %.0f @0,0,0" % (depth / 2, w - 10, h - 10), "MOVE %.1f,0,%.1f $" % (inset + depth / 4, z + 5))
        return name

    def flares(self, extra=38, width=70, depth=60):
        """Dark arch cladding: a half ring round each arch, standing out from the side."""
        names = []
        for tag, x, t in (("F", self.FA, self.front), ("R", self.RA, self.rear)):
            for sgn, side in ((1, "L"), (-1, "R")):
                n = "Flare" + tag + side
                r0 = t["R"] + extra
                hw = self.hw(x)
                y = sgn * (hw + depth * 0.5) if sgn > 0 else -(hw - depth * 0.5)
                self.add(n, "TUBE %.0f %.0f %.0f @0,0,0" % (r0 + width, r0, depth), "ROTATE $ 90,0,0", "MOVE %.1f,%.1f,%.1f $" % (x, y, t["R"]))
                # Only the half above the axle (and a little below it).
                self.out += ["BOX %.0f %.0f %.0f @0,0,0" % (2 * (r0 + width) + 20, depth + 40, r0 + width), "RENAME FlareCut",
                             "MOVE %.1f,%.1f,%.1f FlareCut" % (x, sgn * hw, t["R"] - 40 - (r0 + width)), "BOOLEAN %s SUBTRACT FlareCut" % n]
                names.append(n)
        return names

    def rails(self, x0, x1, y, z, name="Rail"):
        """Roof rails along x0..x1 at +-y, z their foot."""
        names = []
        for sgn, side in ((1, "L"), (-1, "R")):
            self.add(name + side, "BOX %.0f 40 45 @0,0,0" % (x1 - x0), "MOVE %.1f,%.1f,%.1f $" % (0.5 * (x0 + x1), sgn * y, z))
            names.append(name + side)
        return names

    def pair(self, name, make, x, y, z, rot="0,0,0", flip_rot=True):
        """The same part either side: at (x, +-y, z), rotated (yaw mirrored on the right)."""
        names = []
        for sgn, side in ((1, "L"), (-1, "R")):
            rx, ry, rz = (float(v) for v in rot.split(","))
            if flip_rot and sgn < 0: rx, rz = -rx, -rz
            place = ["ROTATE $ %g,%g,%g" % (rx, ry, rz)] if (rx or ry or rz) else []
            self.add(name + side, make, *(place + ["MOVE %.1f,%.1f,%.1f $" % (x, sgn * y, z)]))
            names.append(name + side)
        return names

    # Materials.
    def mat(self, names, *props):
        names = " ".join(names) if isinstance(names, (list, tuple)) else names
        self.out.extend("MATERIAL %s %s" % (names, p) for p in props)

    def glass(self, names, tint="#08090b", transmission=0.4):
        self.mat(names, "glass", "COLOR " + tint, "TRANSMISSION %.2f" % transmission, "ROUGHNESS 0.02")

    def dark(self, names, roughness=0.4):
        self.mat(names, "COLOR #0b0c0e", "ROUGHNESS %.2f" % roughness)

    def lamp(self, names, color, emission=0.6):
        self.mat(names, "COLOR " + color, "EMISSION %.2f" % emission, "ROUGHNESS 0.08")

    def finish(self, paint):
        """Paints the body and mirrors, then turns the car nose to +X."""
        self.mat(self.paint, "COLOR " + paint, "FINISH polished")
        self.out += ["MIRROR %s PLANE YZ AT %.1f,0,0 MOVE" % (n, self.L / 2) for n in self.parts]
        return "\n".join(self.out)

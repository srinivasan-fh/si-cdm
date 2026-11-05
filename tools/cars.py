#!/usr/bin/env python3
"""Writes a sample car's exterior as SI-CDM commands, one body type at a time:

    tools/cars.py supercar > docs/examples/supercar.txt
    tools/cars.py all             (writes every docs/examples/<type>.txt)

Each car is generic, drawn to the overall sizes of a typical car of its type (no make's
styling, badges or names). The shared machinery is tools/carkit.py; here are each car's
lines and details. Sizes in mm.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from carkit import Car  # noqa: E402


# Width factor, roof lift and underbody lift for each car (see Car): measured on the built car.
TUNE = {"supercar": (1.00041, -0.3, -0.3), "sedan": (0.99818, -6.2, 7.8), "hatchback": (0.99759, -11.6, 7.5), "suv-rugged": (0.99874, -49.0, -4.8), "crossover": (0.99777, -42.9, 8.4), "coupe": (1.00503, -2.8, 11.3), "convertible": (1.00358, 0.0, 9.7), "mpv": (0.99793, -46.5, 7.8), "ev-crossover": (0.99758, -6.6, 8.4)}


def supercar():
    """Mid-engined wedge supercar: 4520 long, 1933 wide, 1165 high, 2620 wheelbase, 115 ground
    clearance; 245/30 R20 front, 305/30 R20 rear. Creased sections (polylines, lofted corner to
    corner), a cab-forward glasshouse running back over the engine, six-sided arches, big
    intakes, slim lamps, a wing."""
    L = 4520.0
    c = Car(L, WB=2620.0, FA=1010.0, front="245/30 R20", rear="305/30 R20",
            top=[(0, 470), (150, 560), (500, 680), (1010, 760), (1300, 800), (1900, 850), (2600, 900), (3630, 975), (4100, 975), (4400, 950), (L, 905)],
            dip=[(0, 25), (400, 55), (900, 65), (1250, 30), (1400, 10), (L, 10)],
            bot=[(0, 255), (120, 175), (450, 135), (1010, 123), (2400, 120), (3630, 127), (4200, 175), (L, 335)],
            half=[(0, 703), (120, 844), (450, 934), (1010, 959), (1500, 939), (2000, 909), (2700, 929), (3630, 970.5), (4150, 959), (4400, 909), (L, 854)],
            roof=[(1300, 790), (1650, 975), (2050, 1125), (2350, 1165), (2750, 1140), (3250, 1065), (3700, 1005), (4000, 985)],
            G0=1300.0, G1=4000.0, tune=TUNE["supercar"])
    shape = [(0, -0.006, 0), (0.78, 0, 0), (0.94, 0.12, 0), (1.0, 0.42, 0), (0.985, 0.62, 0), (0.90, 0.86, 0), (0.80, 1.0, 0), (0.42, 1.0, 0.85), (0, 1.0, 1.0)]
    c.body(stations(c), shape, nose=(40, 0.55), tail=(4480, 0.82), crease=True)
    c.glasshouse([1301, 1450, 1650, 1850, 2050, 2350, 2750, 3250, 3700, 3999], [(1, 0), (0.95, 0.25), (0.72, 0.80), (0.45, 0.98)],
                 inset=235, taper=(300, 0.5, 600, 0.65), crease=True)
    c.arches(extra=26, sides=6)
    c.door_lines([1390, 2620], low=150, high=60, sill=(1390, 2620))
    dark, lamps, tails = [], [], []
    for sgn, side in ((1, "L"), (-1, "R")):
        # Corner intakes in the nose, slim slanted headlamps on the wings above them.
        c.add("Intake" + side, "BOX 240 330 120 @0,0,0", "ROTATE $ 0,32,%d" % (sgn * 12), "MOVE 125,%.1f,190 $" % (sgn * 610))
        c.add("Head" + side, "BOX 330 250 36 @0,0,0", "ROTATE $ 0,-14,%d" % (-sgn * 14), "MOVE 330,%.1f,%.1f $" % (sgn * 690, c.zt(330) - 26))
        c.add("Lamp" + side, "BOX 250 26 12 @0,0,0", "ROTATE $ 0,-14,%d" % (-sgn * 14), "MOVE 320,%.1f,%.1f $" % (sgn * 640, c.zt(320) - 2))
        # A big intake in the side behind the door, a dark sill between the wheels.
        c.add("Side" + side, "BOX 620 110 330 @0,0,0", "ROTATE $ 0,0,%d" % (-sgn * 8), "MOVE 2960,%.1f,410 $" % (sgn * (c.hw(2960) - 52)))
        c.add("Sill" + side, "BOX 1500 40 90 @0,0,0", "MOVE 2010,%.1f,%.1f $" % (sgn * (c.hw(2010) - 22), c.zb(2010) + 2))
        # Tail lamps: thin bars at the rear corners; exhausts high in the middle.
        c.add("Tail" + side, "BOX 40 420 36 @0,0,0", "ROTATE $ 0,0,%d" % (sgn * 8), "MOVE %.1f,%.1f,%.1f $" % (L - 40, sgn * 600, c.zt(L - 40) - 90))
        c.add("Pipe" + side, "CYLINDER 46 120 @0,0,0", "ROTATE $ 0,90,0", "MOVE %.1f,%.1f,560 $" % (L - 100, sgn * 130))
        dark += ["Intake" + side, "Side" + side, "Sill" + side]; lamps.append("Lamp" + side); tails.append("Tail" + side)
    c.add("Splitter", "BOX 160 1660 34 @0,0,0", "MOVE 90,0,%.1f $" % (c.zb(90) - 6))
    c.add("RearGrille", "BOX 60 1560 200 @0,0,0", "MOVE %.1f,0,640 $" % (L - 30))
    c.add("Diffuser", "BOX 360 1300 150 @0,0,0", "MOVE %.1f,0,%.1f $" % (L - 170, c.zb(L - 170) - 10))
    # Louvres over the engine, and a wing on two struts.
    for i in range(5):
        x = 3720 + i * 110
        c.add("Louvre%d" % (i + 1), "BOX 50 900 16 @0,0,0", "MOVE %.1f,0,%.1f $" % (x, c.zt(x) - 10))
        dark.append("Louvre%d" % (i + 1))
    c.add("Wing", "BOX 300 1560 24 @0,0,0", "ROTATE $ 0,6,0", "MOVE 4300,0,1090 $")
    for sgn, side in ((1, "L"), (-1, "R")):
        c.add("Strut" + side, "BOX 70 20 140 @0,0,0", "MOVE 4300,%.1f,%.1f $" % (sgn * 420, c.zt(4300) - 30))
        dark.append("Strut" + side)
    c.mirrors(1540, up=10, out=40, size=(90, 100, 46), share=0.84)
    c.wheels(spokes=10, win=40, ring=148, rim_color="#2a2c30", caliper="#d8b400", finish="satin")
    c.glass("Glasshouse", transmission=0.12)
    c.glass(["HeadL", "HeadR"], tint="#16191d", transmission=0.3)
    c.lamp(lamps, "#f4f6ff", 0.9)
    c.lamp(tails, "#c8101a", 0.8)
    c.dark(dark + ["Splitter", "RearGrille", "Diffuser", "Wing"])
    c.mat(["PipeL", "PipeR"], "chrome")
    return c.finish("#e3690f")


# Section shapes (see Car.section): rounded for saloons, upright for SUVs and vans.
ROUND = [(0, 0, 0), (0.70, 0, 0), (0.92, 0.06, 0), (0.985, 0.22, 0), (1.0, 0.47, 0), (0.997, 0.65, 0), (0.982, 0.70, 0),
         (0.94, 0.93, 0), (0.74, 1.0, 0), (0.36, 1.0, 0.8), (0, 1.0, 1.0)]
UPRIGHT = [(0, 0, 0), (0.82, 0, 0), (0.95, 0.035, 0), (0.993, 0.13, 0), (1.0, 0.42, 0), (0.997, 0.78, 0), (0.982, 0.965, 0),
           (0.93, 1.0, 0), (0.5, 1.0, 0.6), (0, 1.0, 1.0)]
SPORT = [(0, 0, 0), (0.72, 0, 0), (0.92, 0.08, 0), (0.985, 0.26, 0), (1.0, 0.52, 0), (0.975, 0.78, 0), (0.90, 0.96, 0),
         (0.74, 1.0, 0), (0.50, 1.0, 0.75), (0.22, 1.0, 1.0), (0, 1.0, 1.0)]
# Glasshouse shapes (see Car.glasshouse).
G_ROUND = [(1, 0), (0.97, 0.30), (0.685, 0.84), (0.384, 0.985)]
G_UPRIGHT = [(1, 0), (0.99, 0.40), (0.87, 0.93), (0.766, 0.995)]
G_MID = [(1, 0), (0.98, 0.35), (0.80, 0.90), (0.62, 0.99)]
G_SPORT = [(1, 0), (0.96, 0.30), (0.713, 0.82), (0.41, 0.985)]


def stations(c, step=330.0):
    """Body stations: close together at the ends (where the body rounds off) and evenly
    spaced between, growing by no more than about twice from one span to the next: the smooth
    loft runs a curve through the sections, and next to much longer spans a short one makes it
    overshoot along the car and fold the surface over."""
    ends = [0.0, 60.0, 180.0, 400.0]
    n = max(1, round((c.L - 800.0) / step))
    mid = [400.0 + (c.L - 800.0) * i / n for i in range(1, n)]
    return ends + mid + [c.L - x for x in reversed(ends)]


def gstations(c):
    xs = sorted({c.G0 + 1, c.G1 - 1} | {float(x) for x, _ in c.roof if c.G0 < x < c.G1})
    return xs


def road(c, doors, sill, bonnet, mirror_x, grille, head, tail, paint, spokes=(5, 60, 138), rim="#c5c8cd", caliper="#2a2c30",
         lid=None, tailgate=None, glass_t=0.35, extras_dark=(), wheel_in=28, arch_extra=32, mirror_share=1.0):
    """What every road car here has: arches, shut lines, a grille, lamps, mirrors, wheels.
    head / tail: (make, x, y, z, rot) for one lamp of the pair (left side)."""
    c.arches(extra=arch_extra)
    c.door_lines(doors, low=95, high=110, sill=sill)
    if bonnet:  # a bonnet: its end across the car and its sides along the fenders
        x0, x1, share = bonnet
        c.cross_line(x1, share)
        c.lid_lines(x0, x1, share)
    if lid:     # a boot lid
        x0, x1, share = lid
        c.cross_line(x0, share)
        c.lid_lines(x0, x1, share)
    if tailgate:  # a tailgate's lower edge across the tail at z
        z, share = tailgate
        c.box_cut(40, 2 * share * c.hw(c.L - 30), 3, c.L - 10, 0, z)
    dark = list(extras_dark)
    if grille: dark.append(c.grille(*grille))
    heads = c.pair("Head", *head)
    tails = c.pair("Tail", *tail)
    c.mirrors(mirror_x, up=25, out=45, share=mirror_share)
    c.wheels(spokes=spokes[0], win=spokes[1], ring=spokes[2], rim_color=rim, caliper=caliper, inset=wheel_in)
    c.glass("Glasshouse", transmission=glass_t)
    c.glass(heads, tint="#1b1e22", transmission=0.3)
    c.lamp(tails, "#c0101a", 0.7)
    c.dark(dark)
    return c.finish(paint)


def sedan():
    """Compact three-box sedan: 4580 long, 1750 wide, 1490 high, 2600 wheelbase, 165 ground
    clearance, 185/55 R16 tyres. Four doors, a bonnet and a separate boot whose lid is cut in."""
    L = 4580.0
    c = Car(L, WB=2600.0, FA=900.0, front="185/55 R16", rear="185/55 R16",
            top=[(0, 640), (120, 720), (450, 800), (900, 860), (1400, 915), (2500, 950), (3500, 975), (4100, 990), (4450, 985), (L, 930)],
            dip=[(0, 8), (400, 20), (900, 22), (1300, 10), (1450, 0), (L, 0)],
            bot=[(0, 370), (150, 260), (500, 195), (900, 175), (2400, 165), (3500, 180), (4100, 240), (4400, 320), (L, 430)],
            half=[(0, 600), (120, 780), (450, 850), (900, 870), (1500, 872), (2500, 870), (3500, 875), (4100, 860), (4400, 820), (L, 720)],
            roof=[(1450, 925), (1850, 1230), (2250, 1450), (2600, 1490), (3100, 1475), (3500, 1330), (3800, 1110), (3950, 1000)],
            G0=1450.0, G1=3950.0, tune=TUNE["sedan"])
    c.body(stations(c), ROUND, nose=(45, 0.6), tail=(L - 70, 0.75))
    c.glasshouse(gstations(c), G_ROUND, inset=95, sink=25, taper=(250, 0.6, 450, 0.80))
    return road(c, doors=[1480, 2470, 3330], sill=(1520, 3290), bonnet=(250, 1380, 0.8), mirror_x=1620,
                grille=(900, 150, 470), head=("ELLIPSOID 90 190 45 @0,0,0", 110, 610, 650, "0,0,-16"),
                tail=("ELLIPSOID 50 210 50 @0,0,0", L - 45, 650, 870, "0,0,12"), lid=(3990, L - 60, 0.8),
                paint="#1f3a68", spokes=(10, 30, 140))


def hatchback():
    """Compact hatchback: 3860 long, 1735 wide, 1520 high, 2450 wheelbase, 163 ground
    clearance, 185/65 R15 tyres. Short overhangs, a tall roof run to the tail, and a tailgate
    that lifts from below the rear lamps."""
    L = 3860.0
    c = Car(L, WB=2450.0, FA=700.0, front="185/65 R15", rear="185/65 R15",
            top=[(0, 640), (120, 720), (400, 790), (700, 840), (1200, 905), (2200, 935), (3150, 965), (3600, 975), (3800, 960), (L, 900)],
            dip=[(0, 8), (300, 15), (700, 18), (1100, 8), (1250, 0), (L, 0)],
            bot=[(0, 360), (150, 260), (400, 195), (700, 175), (2000, 163), (3150, 178), (3550, 250), (L, 400)],
            half=[(0, 610), (120, 780), (400, 850), (700, 862), (1500, 866), (2500, 866), (3150, 867.5), (3550, 850), (3760, 800), (L, 740)],
            roof=[(1250, 915), (1600, 1220), (2000, 1470), (2400, 1520), (3000, 1515), (3400, 1490), (3650, 1380), (3790, 1060)],
            G0=1250.0, G1=3790.0, tune=TUNE["hatchback"])
    c.body(stations(c), ROUND, nose=(45, 0.62), tail=(L - 50, 0.85))
    c.glasshouse(gstations(c), G_UPRIGHT, inset=100, sink=25, taper=(230, 0.65, 180, 0.88))
    return road(c, doors=[1270, 2150, 2950], sill=(1270, 2950), bonnet=(200, 1200, 0.8), mirror_x=1400,
                grille=(760, 170, 440), head=("ELLIPSOID 100 180 55 @0,0,0", 105, 590, 640, "0,0,-20"),
                tail=("ELLIPSOID 45 110 120 @0,0,0", L - 30, 700, 860, "0,0,10"), tailgate=(720, 0.85),
                paint="#c8102e", spokes=(8, 38, 132))


def suv():
    """Rugged three-row SUV: 4662 long, 1917 wide, 1857 high, 2750 wheelbase, 200 ground
    clearance, 255/60 R18 tyres. Boxy and upright, a tall flat bonnet, a big grille, dark arch
    cladding, roof rails, side steps and a skid plate."""
    L = 4662.0
    c = Car(L, WB=2750.0, FA=880.0, front="255/60 R18", rear="255/60 R18",
            top=[(0, 920), (60, 990), (300, 1030), (880, 1060), (1500, 1090), (2700, 1110), (3630, 1120), (4400, 1125), (4600, 1110), (L, 1050)],
            dip=[(0, 6), (L, 6)],
            bot=[(0, 470), (200, 340), (600, 262), (880, 228), (2300, 213), (3630, 228), (4200, 290), (L, 480)],
            half=[(0, 880), (80, 945), (400, 955), (880, 958), (2300, 955), (3630, 958.5), (4400, 950), (4600, 925), (L, 880)],
            roof=[(1520, 1080), (1900, 1560), (2250, 1800), (2500, 1850), (4100, 1857), (4450, 1840), (4560, 1650), (4600, 1150)],
            G0=1520.0, G1=4600.0, tune=TUNE["suv-rugged"])
    c.body(stations(c), UPRIGHT, nose=(50, 0.86), tail=(L - 60, 0.9))
    c.glasshouse(gstations(c), G_UPRIGHT, inset=105, sink=25, taper=(200, 0.72, 120, 0.94))
    flares = c.flares(extra=40)
    rails = c.rails(2300, 4350, 0.70 * (c.hw(3300) - 105), 1857 - 45)
    steps = c.pair("Step", "BOX 1500 90 40 @0,0,0", 2255, c.hw(2255) + 5, c.zb(2255) + 15)
    c.add("Skid", "BOX 120 900 60 @0,0,0", "ROTATE $ 0,-25,0", "MOVE 60,0,%.1f $" % (c.zb(60) + 10))
    c.mat("Skid", "aluminium", "FINISH brushed")
    return road(c, doors=[1560, 2600, 3600], sill=None, bonnet=(120, 1480, 0.86), mirror_x=1660,
                grille=(1100, 330, 600), head=("BOX 60 300 110 @0,0,0", 20, 735, 860, "0,0,0"),
                tail=("BOX 50 90 360 @0,0,0", L - 20, 860, 880, "0,0,0"), tailgate=(830, 0.85),
                paint="#e8e8e6", spokes=(6, 58, 140), rim="#9ea2a7", extras_dark=flares + rails + steps, arch_extra=40, glass_t=0.3)


def crossover():
    """Compact crossover: 4330 long, 1790 wide, 1635 high, 2610 wheelbase, 190 ground
    clearance, 215/60 R17 tyres. A car-like body raised on bigger wheels, dark arch cladding,
    roof rails, split lamps (slim lights over the main lamps), a skid plate."""
    L = 4330.0
    c = Car(L, WB=2610.0, FA=840.0, front="215/60 R17", rear="215/60 R17",
            top=[(0, 800), (100, 880), (400, 950), (840, 990), (1400, 1030), (2500, 1060), (3450, 1080), (4000, 1085), (4250, 1060), (L, 1000)],
            dip=[(0, 10), (400, 22), (840, 24), (1300, 10), (1450, 0), (L, 0)],
            bot=[(0, 420), (150, 300), (500, 230), (840, 205), (2200, 190), (3450, 205), (3950, 270), (L, 440)],
            half=[(0, 690), (100, 840), (400, 885), (840, 893), (2200, 895), (3450, 895.5), (3950, 880), (4200, 840), (L, 780)],
            roof=[(1430, 1020), (1800, 1350), (2200, 1580), (2550, 1635), (3400, 1625), (3850, 1580), (4080, 1420), (4180, 1100)],
            G0=1430.0, G1=4180.0, tune=TUNE["crossover"])
    c.body(stations(c), ROUND, nose=(45, 0.75), tail=(L - 60, 0.82))
    c.glasshouse(gstations(c), G_MID, inset=105, sink=25, taper=(230, 0.65, 250, 0.85))
    flares = c.flares(extra=36)
    rails = c.rails(2250, 3900, 0.62 * (c.hw(3000) - 105), 1635 - 45)
    drl = c.pair("Light", "BOX 40 300 22 @0,0,0", 95, 600, 880, "0,0,-14")
    c.add("Skid", "BOX 90 760 50 @0,0,0", "ROTATE $ 0,-25,0", "MOVE 60,0,%.1f $" % (c.zb(60) + 20))
    c.mat("Skid", "aluminium", "FINISH brushed")
    c.lamp(drl, "#f4f6ff", 0.9)
    return road(c, doors=[1460, 2500, 3420], sill=None, bonnet=(160, 1400, 0.82), mirror_x=1580,
                grille=(820, 260, 520), head=("ELLIPSOID 70 160 50 @0,0,0", 70, 610, 720, "0,0,-14"),
                tail=("ELLIPSOID 45 240 50 @0,0,0", L - 45, 640, 960, "0,0,12"), tailgate=(800, 0.84),
                paint="#5a6b7d", spokes=(5, 62, 136), extras_dark=flares + rails, arch_extra=36)


def coupe():
    """Front-engined fastback coupe: 4811 long, 1916 wide, 1394 high, 2720 wheelbase, 130
    ground clearance, 255/40 R19 tyres. A long bonnet with the cabin set well back, a roof
    sloping in one line to a short tail, a wide low grille, triple tail lamps, twin exhausts."""
    L = 4811.0
    c = Car(L, WB=2720.0, FA=880.0, front="255/40 R19", rear="255/40 R19",
            top=[(0, 640), (120, 730), (450, 810), (880, 860), (1500, 900), (1950, 920), (2700, 940), (3600, 970), (4400, 1000), (4700, 995), (L, 950)],
            dip=[(0, 20), (400, 45), (1000, 50), (1700, 25), (1950, 0), (L, 0)],
            bot=[(0, 330), (150, 220), (450, 160), (880, 140), (2400, 130), (3600, 145), (4300, 210), (L, 400)],
            half=[(0, 640), (120, 820), (450, 920), (880, 945), (1600, 935), (2500, 930), (3600, 958), (4300, 940), (4650, 890), (L, 800)],
            roof=[(1950, 910), (2300, 1180), (2650, 1360), (2950, 1394), (3350, 1360), (3800, 1230), (4250, 1080), (4550, 1005)],
            G0=1950.0, G1=4550.0, tune=TUNE["coupe"])
    c.body(stations(c), SPORT, nose=(45, 0.6), tail=(L - 70, 0.75))
    c.glasshouse(gstations(c), G_SPORT, inset=170, sink=22, taper=(260, 0.55, 500, 0.7))
    pipes = c.pair("Pipe", "CYLINDER 50 120 @0,0,0", L - 100, 520, 260, "0,90,0", flip_rot=False)
    c.mat(pipes, "chrome")
    bars = []
    for i, y in enumerate((560, 640, 720)):  # three upright bars each side
        bars += c.pair("Bar%d" % (i + 1), "BOX 30 40 150 @0,0,0", L - 22, y, 840)
    c.lamp(bars, "#c0101a", 0.8)
    return road(c, doors=[2000, 3150], sill=(2000, 3150), bonnet=(200, 1900, 0.62), mirror_x=2120,
                grille=(1100, 260, 330), head=("ELLIPSOID 110 170 40 @0,0,0", 160, 700, 690, "0,0,-24"),
                tail=("BOX 30 330 24 @0,0,0", L - 26, 640, 935, "0,0,0"), paint="#f2b705", spokes=(5, 62, 140),
                rim="#2a2c30", caliper="#c41e1e", glass_t=0.3, mirror_share=0.95)


def convertible():
    """Two-seat roadster, shown with its hood down: 3915 long, 1735 wide, 1235 high with the
    hood up, 2310 wheelbase, 136 ground clearance, 205/45 R17 tyres. A long bonnet, an open
    cockpit cut into the body with two seats, a steering wheel and roll hoops, and a framed
    windscreen."""
    L = 3915.0
    c = Car(L, WB=2310.0, FA=780.0, front="205/45 R17", rear="205/45 R17",
            top=[(0, 560), (120, 640), (400, 710), (780, 760), (1300, 810), (1650, 830), (2300, 840), (3090, 860), (3600, 860), (3830, 830), (L, 780)],
            dip=[(0, 15), (400, 30), (780, 32), (1300, 15), (1600, 0), (L, 0)],
            bot=[(0, 300), (150, 210), (450, 160), (780, 145), (2000, 136), (3090, 150), (3500, 210), (L, 330)],
            half=[(0, 600), (120, 780), (400, 845), (780, 862), (1500, 850), (2300, 855), (3090, 867.5), (3500, 850), (3800, 800), (L, 730)],
            roof=[(1650, 830), (2000, 840)], G0=1650.0, G1=2000.0, tune=TUNE["convertible"])
    c.body(stations(c), SPORT, nose=(45, 0.6), tail=(L - 70, 0.78))
    # The cockpit: an opening from behind the windscreen to the rear deck, and what is in it.
    zt = c.zt(2350)
    c.box_cut(1050, 2 * (c.hw(2350) - 190), 400, 2350, 0, zt - 330)
    seats = []
    for sgn, side in ((1, "L"), (-1, "R")):
        y = sgn * 330
        c.add("Cushion" + side, "BOX 480 480 110 @0,0,0", "MOVE 2420,%.1f,%.1f $" % (y, zt - 330))
        c.add("Back" + side, "BOX 110 470 560 @0,0,0", "ROTATE $ 0,-14,0", "MOVE 2700,%.1f,%.1f $" % (y, zt - 300))
        c.add("Hoop" + side, "ELLIPSOID 70 200 130 @0,0,0", "MOVE 2880,%.1f,%.1f $" % (y, zt + 40))
        seats += ["Cushion" + side, "Back" + side]
        c.paint.append("Hoop" + side)
    c.add("Dash", "BOX 160 %.0f 120 @0,0,0" % (2 * (c.hw(1900) - 200)), "MOVE 1900,0,%.1f $" % (zt - 120))
    c.add("Wheel", "TORUS 185 18 @0,0,0", "ROTATE $ 0,-65,0", "MOVE 2080,330,%.1f $" % (zt + 30))
    # The windscreen: a glass pane laid back between two pillars, a header rail on top.
    rake = 58  # degrees back from upright
    x0, z0 = 1690.0, c.zt(1690) - 10
    h = 440.0
    xm = x0 + 0.5 * h * math.sin(math.radians(rake))
    zm = z0 + 0.5 * h * math.cos(math.radians(rake))
    ws = 2 * (c.hw(xm) - 150)
    c.add("Windscreen", "BOX 8 %.0f %.0f @0,0,0" % (ws, h), "ROTATE $ 0,%d,0" % rake, "MOVE %.1f,0,%.1f $" % (xm, zm - h / 2))
    frame = c.pair("Pillar", "BOX 40 40 %.0f @0,0,0" % (h + 20), xm, ws / 2 + 10, zm - h / 2 - 10, "0,%d,0" % rake, flip_rot=False)
    xt = x0 + h * math.sin(math.radians(rake))
    zt2 = z0 + h * math.cos(math.radians(rake))
    c.add("Header", "BOX 60 %.0f 30 @0,0,0" % (ws + 60), "MOVE %.1f,0,%.1f $" % (xt, zt2 - 15))
    c.arches(extra=30)
    c.door_lines([1700, 2750], low=95, high=110, sill=(1700, 2750))
    c.cross_line(1600, 0.78)
    c.lid_lines(200, 1600, 0.78)
    c.cross_line(3050, 0.8)
    c.lid_lines(3050, L - 70, 0.8)
    dark = [c.grille(640, 160, 330)]
    heads = c.pair("Head", "ELLIPSOID 120 160 40 @0,0,0", 190, 610, 640, "0,0,-26")
    tails = c.pair("Tail", "ELLIPSOID 50 140 60 @0,0,0", L - 30, 600, 760, "0,0,0")
    c.mirrors(1820, up=30, out=45, share=0.95)
    c.wheels(spokes=10, win=32, ring=140, rim_color="#2a2c30", caliper="#2a2c30")
    c.glass("Windscreen", transmission=0.5)
    c.glass(heads, tint="#1b1e22", transmission=0.3)
    c.lamp(tails, "#c0101a", 0.7)
    c.mat(seats, "COLOR #5b3a29", "ROUGHNESS 0.6")
    c.dark(dark + frame + ["Header", "Dash", "Wheel"])
    return c.finish("#9e1b22")


def mpv():
    """Seven-seat MPV: 4735 long, 1830 wide, 1795 high, 2750 wheelbase, 178 ground clearance,
    215/55 R17 tyres. A short bonnet, a long tall roof over three rows of seats, three side
    windows a side, an upright tail, roof rails."""
    L = 4735.0
    c = Car(L, WB=2750.0, FA=900.0, front="215/55 R17", rear="215/55 R17",
            top=[(0, 820), (100, 900), (400, 960), (900, 1000), (1300, 1030), (2500, 1050), (3650, 1060), (4400, 1065), (4650, 1050), (L, 1000)],
            dip=[(0, 8), (400, 15), (900, 15), (1250, 0), (L, 0)],
            bot=[(0, 430), (150, 310), (500, 230), (900, 195), (2300, 178), (3650, 195), (4250, 270), (L, 450)],
            half=[(0, 700), (100, 860), (400, 905), (900, 912), (2300, 915), (3650, 915), (4300, 905), (4600, 880), (L, 840)],
            roof=[(1250, 1010), (1650, 1400), (2050, 1700), (2400, 1785), (3800, 1795), (4400, 1780), (4580, 1650), (4650, 1120)],
            G0=1250.0, G1=4650.0, tune=TUNE["mpv"])
    c.body(stations(c), UPRIGHT, nose=(45, 0.78), tail=(L - 50, 0.9))
    c.glasshouse(gstations(c), G_UPRIGHT, inset=100, sink=25, taper=(240, 0.65, 130, 0.93))
    rails = c.rails(2300, 4400, 0.75 * (c.hw(3300) - 100), 1795 - 45)
    # Pillars: dark bands across the glass split it into three windows a side.
    pillars = []
    for i, x in enumerate((2330, 3420)):
        pillars += c.pair("Pillar%d" % (i + 1), "BOX 90 20 %.0f @0,0,0" % (c.zr(x) - c.zt(x) - 60), x, c.hw(x) - 100, c.zt(x) - 5)
    c.mat(pillars, "COLOR #0b0c0e", "ROUGHNESS 0.3")
    return road(c, doors=[1290, 2280, 3320], sill=(1290, 3320), bonnet=(150, 1230, 0.84), mirror_x=1420,
                grille=(980, 260, 560), head=("ELLIPSOID 90 220 60 @0,0,0", 80, 660, 830, "0,0,-14"),
                tail=("BOX 50 110 330 @0,0,0", L - 20, 800, 900, "0,0,0"), tailgate=(820, 0.85),
                paint="#9ba3ab", spokes=(10, 30, 142), extras_dark=rails, glass_t=0.3)


def ev():
    """Compact electric crossover: 3994 long, 1811 wide, 1616 high, 2498 wheelbase, 190 ground
    clearance, 215/60 R16 tyres. A closed nose with a light bar across it (no radiator to feed),
    a dark roof, a tail light bar, arch cladding and a charging flap; no exhausts."""
    L = 3994.0
    c = Car(L, WB=2498.0, FA=780.0, front="215/60 R16", rear="215/60 R16",
            top=[(0, 780), (100, 860), (400, 930), (780, 970), (1300, 1010), (2300, 1050), (3278, 1080), (3700, 1090), (3900, 1070), (L, 1010)],
            dip=[(0, 10), (400, 18), (780, 20), (1200, 8), (1350, 0), (L, 0)],
            bot=[(0, 420), (150, 300), (450, 230), (780, 205), (2000, 190), (3278, 205), (3700, 280), (L, 440)],
            half=[(0, 700), (100, 850), (400, 895), (780, 903), (2000, 905), (3278, 905.5), (3700, 890), (3900, 850), (L, 800)],
            roof=[(1330, 1000), (1700, 1350), (2100, 1570), (2450, 1616), (3000, 1600), (3450, 1540), (3750, 1380), (3880, 1100)],
            G0=1330.0, G1=3880.0, tune=TUNE["ev-crossover"])
    c.body(stations(c), ROUND, nose=(45, 0.75), tail=(L - 50, 0.85))
    c.glasshouse(gstations(c), G_MID, inset=105, sink=25, taper=(230, 0.65, 220, 0.85))
    flares = c.flares(extra=38)
    c.add("LightBar", "BOX 30 1250 18 @0,0,0", "MOVE 70,0,840 $")
    c.add("TailBar", "BOX 30 1350 30 @0,0,0", "MOVE %.1f,0,960 $" % (L - 30))
    c.add("Intake", "BOX 80 900 120 @0,0,0", "MOVE 40,0,380 $")
    c.add("Flap", "BOX 160 12 110 @0,0,0", "MOVE 820,%.1f,880 $" % (c.hw(820) - 12))
    c.lamp("LightBar", "#f4f6ff", 0.9)
    c.lamp("TailBar", "#c0101a", 0.8)
    c.mat("Flap", "COLOR #11807a", "FINISH polished")
    return road(c, doors=[1360, 2340, 3200], sill=None, bonnet=(170, 1300, 0.79), mirror_x=1470,
                grille=None, head=("ELLIPSOID 60 150 45 @0,0,0", 70, 620, 720, "0,0,-14"),
                tail=("ELLIPSOID 40 180 50 @0,0,0", L - 30, 690, 920, "0,0,12"), tailgate=(780, 0.84),
                paint="#11807a", spokes=(5, 66, 134), rim="#2a2c30", extras_dark=flares + ["Intake"], arch_extra=38, glass_t=0.2)


CARS = {"supercar": supercar, "sedan": sedan, "hatchback": hatchback, "suv-rugged": suv, "crossover": crossover,
        "coupe": coupe, "convertible": convertible, "mpv": mpv, "ev-crossover": ev}


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else ""
    def text(k):
        f = CARS[k]
        head = "# " + f.__doc__.strip().replace("\n    ", "\n# ").replace("\n", "\n# ")
        return (head + "\n#   RUN docs/examples/%s.txt        (from the repository folder)\n" % k +
                "# Written by tools/cars.py %s (and tools/carkit.py), whose tables hold every size.\n" % k + f() + "\n")
    if which == "all":
        here = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "examples")
        for k in CARS:
            with open(os.path.join(here, k + ".txt"), "w") as fh:
                fh.write(text(k))
    elif which in CARS:
        sys.stdout.write(text(which))
    else:
        sys.exit("usage: cars.py %s|all" % "|".join(CARS))

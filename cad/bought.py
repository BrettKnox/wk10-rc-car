"""Detailed models of the bought parts, built from published dimensions (sources inline).

These replace the plain envelopes in wk10.refs(): the fit checks run against these shapes.
Each function builds the part in its own frame; wk10.py places it. Numbers marked VERIFY
come from a listing or drawing, not a caliper.
"""
import math
from build123d import (Box, Cylinder, Pos, Rot, Plane, Compound, Part, Polyline, RegularPolygon,
                       RectangleRounded, Axis, GeomType, SortBy, make_face, extrude, loft, offset,
                       fillet)


def bx(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def cz(x, y, z0, z1, d):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(d / 2, z1 - z0)


# ---------------------------------------------------------------- gears
def gear(teeth, dp=48.0, width=4.0, bore=3.17, hub_d=0.0, hub_l=0.0):
    """Spur gear, axis on Z, from z=0. Trapezoid teeth on the 48P module: right size and count,
    not an involute (mesh is set with paper on the car, not from this model)."""
    m = 25.4 / dp
    rp = teeth * m / 2
    ra, rf = rp + m, rp - 1.25 * m
    da = 2 * math.pi / teeth
    pts = []
    for i in range(teeth):
        a0 = da * i
        for f, r in ((0.0, rf), (0.12, rf), (0.3, ra), (0.52, ra), (0.7, rf)):
            pts.append((r * math.cos(a0 + f * da), r * math.sin(a0 + f * da)))
    g = extrude(make_face(Polyline(*pts, close=True)), width)
    if hub_d:
        g += cz(0, 0, width, width + hub_l, hub_d)
    return g - cz(0, 0, -1, width + hub_l + 1, bore)


# ---------------------------------------------------------------- wheel
TIRE_OD, TIRE_W = 67.0, 26.0          # Shaluoman listing
RIM_D = 50.0                          # bead seat; typical 26 mm on-road rim is 52 OD over the lip
RIM_OFFSET = 3.0                      # "2080-3mm" / ET3: nut face 3 mm outboard of the rim centre
HEX_AF, HEX_DEPTH = 12.0, 5.0         # VERIFY hex depth: not published, measure the rim
RIM_BOSS = 8.0                        # hub boss thickness, hex pocket enters from the inboard side


def wheel():
    """Foam tire on a 6-spoke rim. Axis on Z, rim centre at z=0, outer (nut) face at +Z.
    Returns (tire, rim). The hex pocket spans z = RIM_OFFSET - RIM_BOSS .. + HEX_DEPTH."""
    w = TIRE_W
    tire = Cylinder(TIRE_OD / 2, w) - Cylinder(RIM_D / 2, w + 2)
    rims = [g.sort_by(SortBy.RADIUS)[-1] for g in tire.edges().filter_by(GeomType.CIRCLE).group_by(Axis.Z)
            if g]
    tire = fillet([rims[0], rims[-1]], 4.0)
    barrel = Cylinder(RIM_D / 2, w - 1) - Cylinder(RIM_D / 2 - 1.6, w + 2)
    lip = Pos(0, 0, w / 2 + 1.0) * (Cylinder(RIM_D / 2 + 1.2, 2.0) - Cylinder(RIM_D / 2 - 1.6, 3))   # outer bead lip
    z1 = RIM_OFFSET
    z0 = z1 - RIM_BOSS
    rim = barrel + lip + cz(0, 0, z0, z1, 19)
    for i in range(6):
        rim += Rot(0, 0, i * 60) * Pos(RIM_D / 4 + 2.5, 0, z1 - 1.5) * Box(RIM_D / 2 - 7, 6, 3)
    hexa = Pos(0, 0, z0 - 0.01) * extrude(RegularPolygon(HEX_AF / math.sqrt(3), 6), HEX_DEPTH)
    rim = rim - hexa - Cylinder(2.7, w * 2)
    return tire, rim


def tube(points, d):
    """Flexible lead along a polyline (straight runs, rounded joints)."""
    from build123d import Solid, Sphere, Vector
    out = None
    for a, b in zip(points, points[1:]):
        va, vb = Vector(*a), Vector(*b)
        seg = Solid.make_cylinder(d / 2, (vb - va).length, Plane(origin=va, z_dir=vb - va))
        out = seg if out is None else out + seg
    for p in points[1:-1]:
        out += Pos(*p) * Sphere(d / 2)
    return out


# ---------------------------------------------------------------- ESP32-DevKitC V4
# Espressif drawing esp32_devkitc_v4_dimensions.pdf: PCB 48.26 x 27.94, WROOM-32 (18 x 25.5)
# overhangs the antenna end by 6.04, 2 x 19 pins at 2.54, centres 1.30 from the long edges and
# 1.29 from the USB end. No mounting holes. PCB thickness 1.6 assumed.
DEVKIT = (48.26, 27.94, 1.6)
DEVKIT_OVERHANG = 6.04


def esp32_devkit():
    """Board in its own frame: PCB bottom at z=0, PCB centred on x, USB end at -X.
    Returns (board, header pins). Headers point down, 8.5 mm under the PCB."""
    L, W, T = DEVKIT
    pcb = bx(-L / 2, L / 2, -W / 2, W / 2, 0, T)
    mx1 = L / 2 + DEVKIT_OVERHANG
    mod = bx(mx1 - 25.5, mx1, -9, 9, T, T + 0.8)
    shield = bx(mx1 - 25.5 + 1.0, mx1 - 7.0, -8, 8, T + 0.8, T + 3.1)
    usb = bx(-L / 2 - 1.0, -L / 2 + 4.5, -3.9, 3.9, T, T + 3.0)
    reg = bx(-L / 2 + 8, -L / 2 + 14, -3, 3, T, T + 1.6)
    pins = []
    for s in (1, -1):
        y = s * (W / 2 - 1.30)
        x0 = -L / 2 + 1.29
        pins.append(bx(x0 - 1.27, x0 + 18 * 2.54 + 1.27, y - 1.27, y + 1.27, -2.5, 0))
        for i in range(19):
            pins.append(cz(x0 + i * 2.54, y, -8.5, -2.5, 0.64))
    return pcb + mod + shield + usb + reg, Compound(pins)


# ---------------------------------------------------------------- 2S hard-case pack
BATT = (138.0, 47.0, 25.0)            # Zeee 5200 2S hard case, VERIFY your pack


def battery():
    """Pack on its own frame: x 0..L, y 0..W, z 0..H; leads leave the +X end."""
    L, W, H = BATT
    case = bx(0, L, 0, W, 0, H)
    case = fillet(case.edges().filter_by(Axis.X), 2.5)
    label = bx(10, L - 10, -0.4, 0.01, 4, H - 4)                     # recessed label panel, -Y face
    case -= label
    return case


# ---------------------------------------------------------------- motor: QuicRun 3650 SD G2 10.5T
# hobbywing.com: 36 x 52.8 mm can, 3.17 shaft, 186 g, 3.5 mm bullets, hall sensor in the end bell.
# Exposed shaft 15 mm (retailer). Face holes: 2 x M3 at 25 mm (inferred 540 pattern, MEASURE).
MOTOR = dict(d=36.0, l=52.8, shaft=3.17, shaft_l=15.0, holes=25.0)


def motor():
    """Own frame: axis on Z, mounting face at z=0, can toward +Z, shaft toward -Z."""
    d, l = MOTOR["d"], MOTOR["l"]
    can = cz(0, 0, 0, l - 6, d) + cz(0, 0, l - 6, l, d - 1.0)          # can + end bell
    can += cz(0, 0, -1.0, 0, 12.5)                                       # front bearing boss
    for a in range(0, 360, 60):                                          # end-bell vents
        v = Rot(0, 0, a) * Pos(11, 0, l - 3) * Box(7, 3, 7)
        can -= v
    for dz in (1, -1):
        can -= Pos(0, dz * MOTOR["holes"] / 2, 0) * Cylinder(1.25, 8)    # M3 tapped holes
    can += cz(0, 0, -MOTOR["shaft_l"], -1.0, MOTOR["shaft"])
    can += bx(-6, 6, -d / 2 - 4, -d / 2 + 2, l - 12, l - 2)             # sensor port (on top when mounted)
    for i, a in enumerate((-20, 0, 20)):                                 # A B C solder tabs + leads
        x = 11 * math.sin(math.radians(a))
        can += cz(x, 11 * math.cos(math.radians(a)), l, l + 6, 4.5)
    return can


# ---------------------------------------------------------------- ESC: QuicRun 10BL60 Sensored G2
# hobbywing.com: 35.0 x 31 x 30.8 mm with the 25 x 25 x 10 fan, 66.5 g, 14 AWG leads 175 mm.
ESC = (35.0, 31.0, 30.8)


def esc():
    """Own frame: x 0..35 (motor leads leave -X, battery leads +X), y 0..31, z 0..30.8 (fan on top)."""
    L, W, H = ESC
    case = bx(0, L, 0, W, 0, H - 10)
    for i in range(6):                                                   # heatsink fins under the fan
        y = 3 + i * 5
        case += bx(4, L - 4, y, y + 1.6, H - 12, H - 10)
    fan = bx((L - 25) / 2, (L + 25) / 2, (W - 25) / 2, (W + 25) / 2, H - 10, H)
    fan -= cz(L / 2, W / 2, H - 11, H + 1, 23)
    fan += cz(L / 2, W / 2, H - 10, H - 1, 9)                             # hub
    for a in range(0, 360, 45):
        fan += Pos(L / 2, W / 2, H - 5) * Rot(0, 0, a) * Pos(7.5, 0, 0) * Rot(20, 0, 0) * Box(7, 3, 1)
    cap = bx(L - 4, L, W / 2 - 6, W / 2 + 6, 2, 12)                       # input capacitor bank
    return case + fan + cap


# ---------------------------------------------------------------- MG996R servo
# components101 drawing: case 40.3 x 20, bottom to case top 36.6, bottom to ears 26.6, ears 53.6
# tip to tip; datasheet 40.7 x 19.7 x 42.9 over the output turret; holes 49.5 x 10 (Power HD std).
def servo(horn_top, spline_off=10.2, horn_r=12.0):
    """Own frame: case bottom z=0, long axis on Y from 0 (spline end) to -40.7, x centred.
    Single-arm horn pointing +X with its top at horn_top."""
    L, W, H = 40.7, 19.7, 36.6
    c = bx(-W / 2, W / 2, -L, 0, 0, H)
    c += cz(0, -spline_off, H, 42.9, 14)                                  # output turret
    c += cz(0, -spline_off, 42.9, horn_top - 3.5, 5.8)                    # spline
    ears = bx(-W / 2 + 1, W / 2 - 1, -L / 2 - 26.8, -L / 2 + 26.8, 26.6, 29.1)
    for s in (1, -1):
        for dx in (5, -5):
            ears -= cz(dx, -L / 2 + s * 49.5 / 2, 25, 30, 4.1)
    c += ears
    c += bx(-3, 3, -L - 4, -L, 3, 7)                                      # lead exit
    horn = cz(0, -spline_off, horn_top - 3.5, horn_top, 12)
    horn += bx(0, horn_r + 4, -spline_off - 3.5, -spline_off + 3.5, horn_top - 3.5, horn_top - 1.5)
    horn -= cz(horn_r, -spline_off, horn_top - 5, horn_top + 1, 3.0)
    return c, horn


# ---------------------------------------------------------------- body shell
# PROTOform Venturi GT (PRM159125): 430 long, 191 wide, 258 WB, 85-90 g painted, wing included.
# Height, overhangs and post positions are not published: the stations below are a stand-in GT
# shape sized to those numbers, with the hood high enough over the front suspension. VERIFY on arrival.
BODY = dict(L=430.0, W=191.0, front_overhang=90.0, wall=0.5)   # 0.5 after forming gives the listed ~87 g
BODY_STATIONS = [  # fraction of length from the nose, half-width fraction, z bottom, z top, corner r
    (0.00, 0.78, 10, 38, 10), (0.05, 0.93, 8, 62, 14), (0.17, 1.00, 8, 84, 18),
    (0.33, 1.00, 8, 88, 22), (0.47, 0.95, 8, 116, 30), (0.62, 0.95, 8, 118, 30),
    (0.78, 1.00, 8, 84, 22), (0.92, 1.00, 8, 80, 18), (1.00, 0.96, 10, 76, 14)]


def body_shell(fa_x, ra_x, r_wheel, arch_in=38.0):
    """Returns (painted shell, windows, wing) in car coordinates, open underneath. Arches cut
    arch_in mm in from each side (tire inner face plus steering room)."""
    L, W = BODY["L"], BODY["W"]
    x0 = fa_x + BODY["front_overhang"]
    sk = [Plane(origin=(x0 - f * L, 0, (z0 + z1) / 2), x_dir=(0, 1, 0), z_dir=(-1, 0, 0)) *
          RectangleRounded(W * hw, z1 - z0, rr) for f, hw, z0, z1, rr in BODY_STATIONS]
    outer = loft(sk)
    shell = outer - offset(outer, -BODY["wall"])
    shell -= bx(x0 - L - 5, x0 + 5, -W, W, -10, 10.5)                    # open underneath
    for x in (fa_x, ra_x):                                               # wheel arches, sides only
        for s in (1, -1):
            yc = s * (W / 2 + 30 - arch_in)
            ra, sl = r_wheel + 9, 12.0                                     # trim line: tire + 9, stretched
            for dx in (-sl, sl):                                           # +-12 fore-aft for steering lock
                shell -= Pos(x + dx, yc, r_wheel) * Rot(90, 0, 0) * Cylinder(ra, 60)
            shell -= bx(x - sl, x + sl, yc - 30, yc + 30, -1, r_wheel + ra)
            shell -= bx(x - ra - sl, x + ra + sl, yc - 30, yc + 30, -1, r_wheel)   # open to the skirt
    st = lambda f: x0 - f * L
    glass = (bx(st(0.465), st(0.36), -W, W, 80, 120) +                   # windscreen
             bx(st(0.62), st(0.48), -W, W, 84, 104) +                    # side glass
             bx(st(0.78), st(0.63), -W * 0.3, W * 0.3, 86, 120))        # rear screen
    windows = shell & glass
    shell = shell - glass
    xw = st(0.965)
    deck = BODY_STATIONS[-2][3]
    wing = bx(xw - 34, xw, -W * 0.46, W * 0.46, deck + 16, deck + 17)
    for s in (1, -1):
        wing += bx(xw - 36, xw + 2, s * W * 0.46 - 0.8, s * W * 0.46, deck - 2, deck + 22)   # end plates
        wing += bx(xw - 22, xw - 10, s * W * 0.28 - 0.8, s * W * 0.28, deck - 4, deck + 16)  # stands
    return shell, windows, wing - outer                                   # stands stop on the deck


def body_inner_z(shell, x, y):
    """Underside of the shell's top skin at (x, y): where a body post meets the body."""
    col = shell & bx(x - 0.5, x + 0.5, y - 0.5, y + 0.5, 0, 200)
    return col.bounding_box().min.Z

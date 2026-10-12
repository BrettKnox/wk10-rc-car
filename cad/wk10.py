"""WK-10: printable 1/10 on-road pan car built around parts already on the shelf.

Run:  C:/Users/bman0/Code/OpsBrain/.venv/Scripts/python.exe C:/Users/bman0/Code/rc-car/cad/wk10.py
Writes every printable part (assembly pose; README gives the print orientation) to cad/out/*.stl + *.step and the
assembly to cad/out/wk10_assembly.step, then runs the fit checks at the bottom.

Frame: X forward, Y left, Z up, ground at Z=0, origin midway between the axles.
All lengths mm. Every number a printer or a vendor can move is a knob in the
PARAMS block; the ones marked VERIFY come from a listing, not a caliper.
"""
from pathlib import Path
from build123d import (Box, Cylinder, Pos, Rot, Plane, Compound, mirror,
                       export_stl, export_step, Axis)

OUT = Path(__file__).parent / "out"

# ---------------------------------------------------------------- PARAMS
WB, TRACK = 257.0, 175.0            # wheelbase, wheel-centre track (TT-02 class)
R_W, W_W = 33.5, 26.0               # Shaluoman B09CPJV6VT listing: 67 mm OD, 26 mm wide, 3 mm offset
FA_X, RA_X = WB / 2, -WB / 2
AXLE_Z = R_W

FLOOR_Z, PLATE_T = 5.0, 4.0         # ride height under the pan, pan thickness
TOP = FLOOR_Z + PLATE_T
HALF_W = 52.0                       # pan half-width
PAN_X0, PAN_X1 = -88.0, 152.0
JOINT_X = 86.0                      # pan splits here: both halves fit a 220 bed
RAIL_T, RAIL_H = 3.0, 8.0

# printer compensation (mechcalc print_hole: holes print ~0.2 under, ASSUMED until
# the bore coupon is measured with the Kynup caliper)
HOLE_COMP = 0.2
B685 = (5.0, 11.0, 5.0)             # 685ZZ from the 68 pc kit: bore, OD, width
BRG_OD = B685[1] + HOLE_COMP        # light press
ROD3_SLIDE = 3.0 + 0.3 + HOLE_COMP  # kingpin slides in the knuckle
ROD3_PRESS = 3.0                    # prints ~2.8: drill to 3.0 for a press
M3_CLR, M3_INS, M3_INS_DEPTH = 3.4, 4.2, 6.3
M4_SHOULDER = 5.0 + 0.15 + HOLE_COMP  # uxcell shoulder bolt is 5 mm, not 4

# drivetrain: 48P pinion/spur, 540 can motor
PINION_T, SPUR_T, DP = 16, 87, 48.0
CENTRE = (PINION_T + SPUR_T) / DP / 2 * 25.4      # 27.25 mm
SPUR_D = SPUR_T / DP * 25.4                        # 46.0 mm
MOTOR_D, MOTOR_L, MOTOR_HOLE_SPC = 36.0, 53.0, 25.0  # 3650 can (QR3650 36x52.8); 540 std M3 @ 25, MEASURE yours
MOTOR_X = RA_X - CENTRE            # behind the axle: clears the pod pivot, loads the driven wheels

# steering: MG996R (VERIFY body 40.7 x 19.7 x 42.9, ears 54.5 wide, holes 49.5 x 10)
SERVO = (19.7, 40.7, 42.9)          # X, Y, Z as mounted (long axis across the car)
SERVO_EAR_Z, SERVO_HOLE_SPC = 26.5, 49.5
SERVO_X = 86.0
SPLINE_OFF = 10.2                   # spline centre from the near end of the case

KINGPIN_Y = 56.0                    # inboard of the bearing housing
KNUCKLE_Z = (22.0, 36.0)            # barrel on the kingpin: spring gap above, droop room below
BUMP_MM, DROOP_MM = 7.0, 5.0        # travel the checks prove clear; bump_stop sleeve enforces 7
MAX_LOCK_DEG = 28.0                 # firmware endpoint; the sweep check proves it clears
KP_Z0, KP_Z1 = 9.0, 56.0            # 3 mm rod, cut 47 mm
LOWER_Z = (TOP, TOP + 6.0)     # sits on the pan top
UPPER_Z = (50.0, 55.0)              # 14 mm spring gap; housing clears the arm through bump

POD_PLATE_Y, POD_T = 45.0, 5.0      # pod side plates (PETG: motor runs hot)
PIVOT_X, PIVOT_Z = -78.0, 16.0
POD_BRACE_X = RA_X + 8              # brace sits between the plates, ahead of the axle

BATT = (138.0, 47.0, 25.0)          # 2S hard case, VERIFY the pack you buy
BATT_X0 = -66.0


# ---------------------------------------------------------------- helpers
def bx(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def cz(x, y, z0, z1, d):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(d / 2, z1 - z0)


def cy(x, z, y0, y1, d):
    return Pos(x, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(d / 2, abs(y1 - y0))


def cx(y, z, x0, x1, d):
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(d / 2, abs(x1 - x0))


def insert(x, y, z_top, down=True):
    """Heat-set insert pilot opening at z_top (pointing down) or z_top (pointing up)."""
    z0, z1 = (z_top - M3_INS_DEPTH, z_top + 0.01) if down else (z_top - 0.01, z_top + M3_INS_DEPTH)
    return cz(x, y, z0, z1, M3_INS)


def mirror_y(s):
    return mirror(s, Plane.XZ)


# ---------------------------------------------------------------- pan (chassis)
def pan():
    p = bx(PAN_X0, PAN_X1, -HALF_W, HALF_W, FLOOR_Z, TOP)
    # side rails: most of the pan's bending stiffness lives here
    for s in (1, -1):
        y0, y1 = sorted((s * (HALF_W - RAIL_T), s * HALF_W))
        p += bx(-70, 100, y0, y1, TOP, TOP + RAIL_H)
    p += bx(-70, 72, -1.0, 1.0, TOP, TOP + RAIL_H)           # centre rib = battery wall
    # pod pivot lugs
    for s in (1, -1):
        y0, y1 = sorted((s * 14, s * 24))
        p += bx(PAN_X0, PIVOT_X + 6, y0, y1, TOP, PIVOT_Z + 6)
        p -= cy(PIVOT_X, PIVOT_Z, y0 - 1, y1 + 1, M4_SHOULDER)
    # rear spring brace: post at Y+25, arm back over the pod
    p += bx(PAN_X0, -80, 20, 30, TOP, 44)
    p += bx(-122, -80, 20, 30, 40, 44)
    p -= cz(-116, 25, 39, 45, M3_CLR)
    # front-end bolt pattern (inserts live in the front end, bolts come up from below)
    for x in (112, 140):
        for y in (-30, 30):
            p -= cz(x, y, FLOOR_Z - 1, TOP + 1, M3_CLR)
            p -= cz(x, y, FLOOR_Z - 1, FLOOR_Z + 1.8, 6.0)    # button-head counterbore
    # splice block bolts either side of the joint
    for x in (JOINT_X - 7, JOINT_X + 7):
        for y in (-40, -22, 22, 40):
            p -= cz(x, y, FLOOR_Z - 1, TOP + 1, M3_CLR)
            p -= cz(x, y, FLOOR_Z - 1, FLOOR_Z + 1.8, 6.0)
    # battery strap slots (20 mm velcro) and ESC / RX tie-down slots
    for x in (-40, 40):
        p -= bx(x - 1.5, x + 1.5, 4, 46, FLOOR_Z - 1, TOP + 1)
    for x in (-30, -5, 20, 45):
        p -= bx(x - 1.5, x + 1.5, -46, -30, FLOOR_Z - 1, TOP + 1)
    # chamfer the nose so it climbs kerbs instead of digging in
    p -= Pos(PAN_X1, 0, FLOOR_Z) * Rot(0, 35, 0) * Box(12, 2 * HALF_W + 2, 6)
    return p


def pan_halves():
    full = pan()
    rear = full & bx(PAN_X0 - 5, JOINT_X, -60, 60, 0, 60)
    front = full & bx(JOINT_X, PAN_X1 + 5, -60, 60, 0, 60)
    return rear, front


def splice_servo_block():
    """Bridges the pan joint AND carries the steering servo."""
    sx, sy, sz = SERVO
    b = bx(JOINT_X - 12, JOINT_X + 12, -HALF_W + RAIL_T, HALF_W - RAIL_T, TOP, TOP + 8)
    for x in (JOINT_X - 7, JOINT_X + 7):
        for y in (-40, -22, 22, 40):
            b -= insert(x, y, TOP, down=False)
    # servo: spline on the centreline, case runs to -Y
    y_near = SPLINE_OFF
    y0, y1 = y_near - sy, y_near
    b -= bx(SERVO_X - sx / 2 - 0.3, SERVO_X + sx / 2 + 0.3, y0 - 0.3, y1 + 0.3, TOP - 1, TOP + 9)
    # ear posts rise to the ear underside
    ymid = (y0 + y1) / 2
    for s in (1, -1):
        py = ymid + s * SERVO_HOLE_SPC / 2
        post = bx(SERVO_X - 5, SERVO_X + 5, py - 3.5, py + 3.5, TOP, TOP + SERVO_EAR_Z)
        post -= insert(SERVO_X, py, TOP + SERVO_EAR_Z)
        b += post
    return b


# ---------------------------------------------------------------- front end
def front_end():
    """Base + centre tower + both lower arms, one print, kingpins pressed in."""
    f = bx(104, 150, -40, 40, TOP, TOP + 5)
    f += bx(FA_X - 9, FA_X + 9, -30, 30, TOP, UPPER_Z[0])          # tower
    for s in (1, -1):
        y0, y1 = sorted((s * 30, s * (KINGPIN_Y + 7)))
        arm = bx(FA_X - 10, FA_X + 10, y0, y1, LOWER_Z[0], LOWER_Z[1])
        arm += cz(FA_X, s * KINGPIN_Y, LOWER_Z[0], LOWER_Z[1], 14)
        arm -= cz(FA_X, s * KINGPIN_Y, LOWER_Z[0] - 1, LOWER_Z[1] + 1, ROD3_PRESS)
        f += arm
        # upper-arm inserts on the tower top
        for x in (FA_X - 5, FA_X + 5):
            f -= insert(x, s * 22, UPPER_Z[0])
    for x in (112, 140):
        for y in (-30, 30):
            f -= insert(x, y, TOP, down=False)
    f += bx(150, 158, -40, 40, TOP, TOP + 12)                       # bumper
    # body posts (6 mm OD, clip holes every 5 mm drilled to taste)
    for y in (-20, 20):
        f += cz(144, y, TOP + 5, 90, 6.0)
        f -= cz(144, y, TOP + 5, 91, 2.6)
    return f


def upper_arm(side=1):
    s = side
    y0, y1, y2 = s * 17, s * 38, s * (KINGPIN_Y + 7)
    a = bx(FA_X - 9, FA_X + 9, min(y0, y1), max(y0, y1), *UPPER_Z)
    # outer half stepped forward so the steering-arm post clears it at full lock
    a += bx(FA_X - 4, FA_X + 9, min(y1, y2), max(y1, y2), *UPPER_Z)
    a += cz(FA_X, s * KINGPIN_Y, *UPPER_Z, 12)
    a -= cz(FA_X, s * KINGPIN_Y, UPPER_Z[0] - 1, UPPER_Z[1] + 1, ROD3_PRESS)
    for x in (FA_X - 5, FA_X + 5):
        a -= cz(x, s * 22, UPPER_Z[0] - 1, UPPER_Z[1] + 1, M3_CLR)
    return a


def knuckle(side=1):
    """Slides and turns on the 3 mm kingpin; two 685ZZ carry the 5 mm stub axle."""
    s = side
    kz0, kz1 = KNUCKLE_Z
    k = cz(FA_X, s * KINGPIN_Y, kz0, kz1, 10)
    y_in, y_out = sorted((s * 60, s * 73))
    k += cy(FA_X, AXLE_Z, y_in, y_out, 16)
    yb0, yb1 = sorted((s * KINGPIN_Y, s * 64))
    k += bx(FA_X - 5, FA_X + 5, yb0, yb1, kz0, kz1)
    # steering arm: back toward the rear axle (Ackermann line), up to tie-rod height
    arm_x = FA_X - 16
    arm_y = s * (KINGPIN_Y - 16 * KINGPIN_Y / WB)
    ya0, ya1 = sorted((s * KINGPIN_Y, arm_y))
    k += bx(arm_x, FA_X, ya0 - 3, ya1 + 3, kz1 - 4, kz1)
    k += cz(arm_x, arm_y, kz1 - 4, 49, 7)
    k -= cz(arm_x, arm_y, kz1 - 5, 50, 2.5)                         # M3 self-tap for the ball stud
    k -= cz(FA_X, s * KINGPIN_Y, kz0 - 1, kz1 + 1, ROD3_SLIDE)
    # bearing seats from each end, 3 mm land between them
    k -= cy(FA_X, AXLE_Z, y_in - 1, y_in + B685[2], BRG_OD) if s > 0 else \
         cy(FA_X, AXLE_Z, y_out - B685[2], y_out + 1, BRG_OD)
    k -= cy(FA_X, AXLE_Z, y_out - B685[2], y_out + 1, BRG_OD) if s > 0 else \
         cy(FA_X, AXLE_Z, y_in - 1, y_in + B685[2], BRG_OD)
    k -= cy(FA_X, AXLE_Z, y_in - 1, y_out + 1, 7.0)
    return k


# ---------------------------------------------------------------- rear pod
def pod_plate(side=1):
    """Side plate: one 685ZZ for the axle. The right (-Y) plate also mounts the motor."""
    s = side
    y0, y1 = sorted((s * POD_PLATE_Y, s * (POD_PLATE_Y + POD_T)))
    p = bx(MOTOR_X - 20, RA_X + 18, y0, y1, 12, 52)                  # sits on the floor
    p -= cy(RA_X, AXLE_Z, y0 - 1, y1 + 1, BRG_OD)
    if s < 0:
        # motor boss clearance + two M3 slots: swinging the slot about the axle sets mesh
        p -= cy(MOTOR_X, AXLE_Z, y0 - 1, y1 + 1, 13.5)
        for dz in (MOTOR_HOLE_SPC / 2, -MOTOR_HOLE_SPC / 2):
            for dx in (-1.5, 0, 1.5):
                p -= cy(MOTOR_X + dx, AXLE_Z + dz, y0 - 1, y1 + 1, M3_CLR)
        p -= cy(MOTOR_X, AXLE_Z, y0 - 1, y1 + 1, 13.5)
    else:
        p -= cy(MOTOR_X, AXLE_Z, y0 - 1, y1 + 1, 24)                 # lightening
    p -= cy(POD_BRACE_X + 5, 48, y0 - 1, y1 + 1, M3_CLR)              # brace bolt
    # bolt tabs to the pod floor
    for x in (MOTOR_X - 12, RA_X + 12):
        p -= cz(x, s * (POD_PLATE_Y + POD_T / 2), 9, 18, M3_CLR)
    return p


def pod_floor():
    f = bx(MOTOR_X - 20, PAN_X0 - 2.5, -POD_PLATE_Y - POD_T, POD_PLATE_Y + POD_T, 8, 12)
    # motor can and spur clearance
    f -= bx(MOTOR_X - 19, MOTOR_X + 19, -POD_PLATE_Y, 6, 11, 13)
    for s in (1, -1):
        for x in (MOTOR_X - 12, RA_X + 12):
            f -= insert(x, s * (POD_PLATE_Y + POD_T / 2), 12)
    # plates register in shallow grooves
    # pivot lug: rides between the pan lugs on the M4 shoulder bolt
    # Each uxcell M4 shoulder bolt (5 x 10 mm shoulder) passes through a 10 mm pan lug
    # and threads into an M4 heat-set insert here, so the shoulder is the bearing.
    f += bx(PAN_X0 - 2.5, PIVOT_X + 6, -13.6, 13.6, TOP + 1.8, PIVOT_Z + 6)  # clears the pan through +-4 deg
    f += bx(PAN_X0 - 8, PAN_X0 - 2.5, -13.6, 13.6, 8, TOP + 3)    # tongue to the floor
    for s in (1, -1):
        y0, y1 = sorted((s * 13.7, s * 6.0))
        f -= cy(PIVOT_X, PIVOT_Z, y0, y1, 5.6)                        # M4 insert pilot
    # spring seat under the brace arm + insert for the M3x35 spring bolt
    f += cz(-116, 25, 12, 15, 10)
    f -= insert(-116, 25, 15)
    # rear top brace ties the two plates so the pod does not lozenge
    return f


def pod_brace():
    bx0 = POD_BRACE_X
    b = bx(bx0, bx0 + 10, -POD_PLATE_Y, POD_PLATE_Y, 44, 52)
    for s in (1, -1):                                                # side inserts, bolts through the plates
        y0, y1 = sorted((s * POD_PLATE_Y, s * (POD_PLATE_Y - M3_INS_DEPTH)))
        b -= cy(bx0 + 5, 48, y0 - 0.01, y1, M3_INS)
    for y in (-25, 25):                                              # rear body posts
        b += cz(bx0 + 5, y, 52, 95, 6.0)
        b -= cz(bx0 + 5, y, 52, 96, 2.6)
    return b


def spool():
    """Locked rear hub: spur bolts to the flange, M3 pinch bolt grips the 5 mm D-shaft."""
    y0, y1 = -POD_PLATE_Y - POD_T - 1.0, -POD_PLATE_Y - POD_T - 19.0
    fl = -POD_PLATE_Y - POD_T - 10.0                                 # spur seats inboard of this face
    s = cy(RA_X, AXLE_Z, y1, y0, 12)
    s += cy(RA_X, AXLE_Z, fl - 3, fl, 30)                            # flange
    s -= cy(RA_X, AXLE_Z, y1 - 1, y0 + 1, 5.2)
    for a in range(0, 360, 120):                                     # spur bolt circle
        import math
        r = 10.0
        s -= cy(RA_X + r * math.cos(math.radians(a)), AXLE_Z + r * math.sin(math.radians(a)),
                fl - 4, fl + 1, M3_CLR)
    s -= cx(y1 + 3, AXLE_Z, RA_X - 7, RA_X + 7, M3_CLR)              # cross pinch bolt, outboard of the spur
    return s


def hex_hub():
    """Replaces the Hobbypark hubs, which need an M4-threaded axle tip that plain 5 mm rod
    does not have. 12 mm hex into the rim, collar inboard, D-bore + M3 bolt as a set screw
    against the flat (grind the flat with the rotary tool). Modelled at the origin, bore on Z."""
    import math
    from build123d import RegularPolygon, extrude
    hexa = extrude(RegularPolygon(12.0 / math.sqrt(3), 6, major_radius=True), 5.0)  # 12 AF
    h = Pos(0, 0, 7) * hexa + cz(0, 0, 0, 7, 16.0)                     # collar under the hex
    h -= cz(0, 0, -1, 13, 5.0 + 0.15 + HOLE_COMP)
    h -= cx(0, 3.5, 0, 9, M3_CLR).rotate(Axis.Z, 90)                   # radial M3 set bolt
    h -= bx(-3.3, 3.3, 3.6, 6.3, -1, 6.4)                              # nut slot (flats up), loads from below
    return h


def bump_stop():
    """Sleeve on the kingpin inside the spring: knuckle hits it after BUMP_MM of travel.
    Needs a spring with ID over 5.4 mm. Modelled at the origin, print 2."""
    gap = UPPER_Z[0] - KNUCKLE_Z[1]
    return cz(0, 0, 0, gap - BUMP_MM, 5.0) - cz(0, 0, -1, gap, ROD3_SLIDE)


# ---------------------------------------------------------------- electronics trays
def esp32_tray():
    """Dev board (VERIFY ~51x28) on 4 standoff pads; zip-tie slots match the pan slots."""
    x0, x1, y0, y1 = 12, 66, -46, -14
    t = bx(x0, x1, y0, y1, TOP, TOP + 2)
    for x in (x0 + 3, x1 - 3):
        for y in (y0 + 3, y1 - 3):
            t += cz(x, y, TOP + 2, TOP + 6, 6)
            t -= cz(x, y, TOP + 1, TOP + 7, 2.5)
    t += bx(x0, x1, y0, y0 + 1.6, TOP, TOP + 12)                     # splash wall outboard
    return t


# ---------------------------------------------------------------- reference bodies (not printed)
def wheel(x, y):
    return cy(x, AXLE_Z, y - W_W / 2, y + W_W / 2, 2 * R_W)


def refs():
    r = {}
    r["wheels"] = Compound([wheel(x, s * TRACK / 2) for x in (FA_X, RA_X) for s in (1, -1)])
    r["battery"] = bx(BATT_X0, BATT_X0 + BATT[0], 1.2, 1.2 + BATT[1], TOP, TOP + BATT[2])
    r["motor"] = cy(MOTOR_X, AXLE_Z, -POD_PLATE_Y, -POD_PLATE_Y + MOTOR_L, MOTOR_D)
    r["spur"] = cy(RA_X, AXLE_Z, -POD_PLATE_Y - POD_T - 5, -POD_PLATE_Y - POD_T - 10, SPUR_D)
    r["pinion"] = cy(MOTOR_X, AXLE_Z, -POD_PLATE_Y - POD_T - 1, -POD_PLATE_Y - POD_T - 11, PINION_T / DP * 25.4)
    sx, sy, sz = SERVO
    r["servo"] = bx(SERVO_X - sx / 2, SERVO_X + sx / 2, SPLINE_OFF - sy, SPLINE_OFF, TOP, TOP + sz)
    r["esc"] = bx(-42, 0, -46, -10, TOP, TOP + 33)                   # VERIFY ESC size
    r["rear_axle"] = cy(RA_X, AXLE_Z, -TRACK / 2 - 8, TRACK / 2 + 8, 5)
    r["kingpins"] = Compound([cz(FA_X, s * KINGPIN_Y, KP_Z0, KP_Z1, 3) for s in (1, -1)])
    return r


# ---------------------------------------------------------------- build
def parts():
    rear, front = pan_halves()
    return {
        "pan_rear": (rear, "PLA, flat, 5 walls, 40% gyroid"),
        "pan_front": (front, "PLA, flat, 5 walls, 40% gyroid"),
        "splice_servo_block": (splice_servo_block(), "PLA, flat"),
        "front_end": (front_end(), "PETG or PLA, base down, 6 walls"),
        "upper_arm_L": (upper_arm(1), "PLA, flat"),
        "upper_arm_R": (upper_arm(-1), "PLA, flat"),
        "knuckle_L": (knuckle(1), "PETG, axle bore vertical, 100% infill"),
        "knuckle_R": (knuckle(-1), "PETG, axle bore vertical, 100% infill"),
        "pod_plate_L": (pod_plate(1), "PETG, flat, 100% infill"),
        "pod_plate_R_motor": (pod_plate(-1), "PETG, flat, 100% infill (motor heat)"),
        "pod_floor": (pod_floor(), "PETG, flat"),
        "pod_brace": (pod_brace(), "PLA, flat"),
        "spool": (spool(), "PETG, flange down, 100% infill"),
        "esp32_tray": (esp32_tray(), "PLA, flat"),
        "hex_hub_x4": (Pos(0, -150, 0) * hex_hub(), "PETG, collar down, 100% infill, print 4"),
        "bump_stop_x2": (Pos(30, -150, 0) * bump_stop(), "PETG, upright, print 2"),
    }


def checks(ps, r):
    """Fit checks that fail loudly if a knob change breaks the car."""
    def gap(a, b):
        return a.distance_to(b)
    solid = {k: v[0] for k, v in ps.items()}
    assert CENTRE > MOTOR_D / 2 - 5, "motor too close to axle"
    assert AXLE_Z - SPUR_D / 2 > 6, f"spur ground clearance {AXLE_Z - SPUR_D / 2:.1f} mm"
    assert AXLE_Z - MOTOR_D / 2 > 10, "motor can too low"
    assert gap(r["motor"], solid["pod_floor"]) > 0.5, "motor hits pod floor"
    assert gap(r["battery"], solid["splice_servo_block"]) > 0.5, "battery hits servo block"
    assert gap(r["servo"], r["battery"]) > 0.5, "servo hits battery"
    for k in ("pan_rear", "pan_front", "front_end"):
        assert gap(r["wheels"], solid[k]) > 3, f"wheel rubs {k}"
    assert gap(r["spur"], solid["pod_plate_R_motor"]) > 0.5, "spur rubs motor plate"
    assert gap(r["esc"], r["battery"]) > 0.5, "esc hits battery"
    # every printed part fits the Ender 3 V3 bed in at least one orientation
    for k, (s, _) in ps.items():
        bb = sorted([s.bounding_box().size.X, s.bounding_box().size.Y, s.bounding_box().size.Z])
        assert bb[1] <= 215 and bb[2] <= 215 * 1.41 and bb[0] <= 245, f"{k} too big {bb}"
        assert s.is_valid, f"{k} invalid solid"
    # printed parts must not overlap each other in the assembled pose
    names = list(solid)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            v = (solid[a] & solid[b]).volume
            assert v < 1.0, f"{a} overlaps {b} by {v:.1f} mm3"
    # steering sweep: wheel + knuckle turn about each kingpin to full lock both ways
    fixed = [solid[k] for k in ("front_end", "upper_arm_L", "upper_arm_R", "pan_front",
                                "splice_servo_block")]
    worst = 99.0
    for s, kn in ((1, "knuckle_L"), (-1, "knuckle_R")):
        whl = wheel(FA_X, s * TRACK / 2)
        for deg in (-MAX_LOCK_DEG, -MAX_LOCK_DEG / 2, MAX_LOCK_DEG / 2, MAX_LOCK_DEG):
            ax = Axis((FA_X, s * KINGPIN_Y, 0), (0, 0, 1))
            for moving in (whl, solid[kn]):
                m = moving.rotate(ax, deg)
                for f in fixed:
                    worst = min(worst, m.distance_to(f))
    assert worst > 0.8, f"steering sweep clearance {worst:.2f} mm at <= {MAX_LOCK_DEG} deg"
    # suspension travel: knuckle + wheel slide on the kingpin; also bump at full lock
    travel = 99.0
    for s, kn in ((1, "knuckle_L"), (-1, "knuckle_R")):
        ax = Axis((FA_X, s * KINGPIN_Y, 0), (0, 0, 1))
        for dz in (BUMP_MM, -DROOP_MM):
            for deg in (0, MAX_LOCK_DEG, -MAX_LOCK_DEG):
                for moving in (wheel(FA_X, s * TRACK / 2), solid[kn]):
                    m = Pos(0, 0, dz) * moving.rotate(ax, deg)
                    for f in fixed:
                        travel = min(travel, m.distance_to(f))
    assert travel > 0.5, f"front travel clearance {travel:.2f} mm"
    # rear pod swings about the pivot: +-4 deg is about +-7 mm at the axle
    pod = [solid[k] for k in ("pod_floor", "pod_plate_L", "pod_plate_R_motor", "pod_brace", "spool")]
    pod += [r["motor"], r["spur"], r["wheels"]]
    pax = Axis((PIVOT_X, 0, PIVOT_Z), (0, 1, 0))
    pod_gap = min(p.rotate(pax, a).distance_to(solid["pan_rear"]) for p in pod for a in (-4, 4))
    assert pod_gap > 0.3, f"pod swing clearance {pod_gap:.2f} mm"
    print(f"checks ok: clearances, spur/motor height, bed fit, validity, no part overlaps, "
          f"steering sweep +-{MAX_LOCK_DEG} deg min gap {worst:.1f} mm, front travel +{BUMP_MM}/-{DROOP_MM} "
          f"min gap {travel:.1f} mm, pod swing +-4 deg min gap {pod_gap:.1f} mm "
          f"(NOT tested: real rim hex depth/offset, tie-rod path, spring solid height)")


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    ps, r = parts(), refs()
    checks(ps, r)
    import json
    manifest = {}
    for name, (shape, note) in ps.items():
        export_stl(shape, str(OUT / f"{name}.stl"))
        export_step(shape, str(OUT / f"{name}.step"))
        bb = shape.bounding_box().size
        manifest[name] = {"print": note, "volume_cm3": round(shape.volume / 1000, 1),
                          "bbox_mm": [round(bb.X, 1), round(bb.Y, 1), round(bb.Z, 1)]}
    asm = Compound([s for k, (s, _) in ps.items() if "_x" not in k] + list(r.values()))  # loose small parts left out
    export_step(asm, str(OUT / "wk10_assembly.step"))
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=1))
    print(json.dumps(manifest, indent=1))

"""WK-10: printable 1/10 on-road pan car built around parts already on the shelf.

Run:  python cad/wk10.py
Writes every printable part (assembly pose; README gives the print orientation) to cad/out/*.stl + *.step and the
assembly to cad/out/wk10_assembly.step, then runs the fit checks at the bottom.

Frame: X forward, Y left, Z up, ground at Z=0, origin midway between the axles.
All lengths mm. Every number a printer or a vendor can move is a knob in the
PARAMS block; the ones marked VERIFY come from a listing, not a caliper.
"""
from functools import lru_cache
from pathlib import Path
import bought
from build123d import (Box, Cylinder, Pos, Rot, Plane, Compound, mirror,
                       export_stl, export_step, Axis)

OUT = Path(__file__).parent / "out"

# ---------------------------------------------------------------- PARAMS
WB, TRACK = 257.0, 164.0            # wheelbase, wheel-centre track: 190 mm over the tires, so a 190 mm
                                    # 1/10 GT body (Venturi GT, 258 WB) covers the wheels
R_W, W_W = 33.5, 26.0               # Shaluoman B09CPJV6VT listing: 67 mm OD, 26 mm wide, 3 mm offset
FA_X, RA_X = WB / 2, -WB / 2
AXLE_Z = R_W

FLOOR_Z, PLATE_T = 5.0, 4.0         # ride height under the pan, pan thickness
TOP = FLOOR_Z + PLATE_T
HALF_W = 52.0                       # pan half-width
PAN_X0, PAN_X1 = -88.0, 152.0
JOINT_X = 86.0                      # pan splits here: both halves fit a 220 bed
RAIL_T, RAIL_H = 3.0, 8.0
PAN_FRONT_HW = 45.0                 # pan half-width around the front axle (tire swing at lock)

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
PINION_T, SPUR_T, DP = 14, 87, 48.0   # 14T for the 3600Kv QR3650: 6.21:1, slots fit 13-17T
CENTRE = (PINION_T + SPUR_T) / DP / 2 * 25.4      # 26.72 mm for 14T
SPUR_D = SPUR_T / DP * 25.4                        # 46.0 mm
MOTOR_D, MOTOR_L, MOTOR_HOLE_SPC = 36.0, 53.0, 25.0  # 3650 can (QR3650 36x52.8); 540 std M3 @ 25, MEASURE yours
MOTOR_X = RA_X - CENTRE            # behind the axle: clears the pod pivot, loads the driven wheels

# steering (Rev F): Injora INJS014 low-profile servo, Kimbrough 114 saver, printed bar with two
# 5.8 mm ball studs, Hot Racing cups on cut M3 rods. The servo sinks through the pan.
SERVO_X = 98.0                      # ahead of the pan joint (86): the joint stays whole under the splice
SERVO_Z0 = FLOOR_Z + 1.0            # servo bottom on a 1 mm skin under the pan
SAVER_R = 14.0                      # VERIFY: saver arm hole radius used for the bar bolt
BALL_R = 20.0                       # bar carries both ball studs this far ahead of the spline
BAR_Y = 7.0                         # ball studs either side of the centreline (cups 8.5 wide)
KNUCKLE_Z_TOP = 48.0                # barrel top; the arm's high run sits over the stub collar
ARM_TIP_TOP = 36.5                  # knuckle ball stud seat: ball centre 3.8 above, level with the bar's

KP_TRAIL = 10.5                     # kingpin ahead of the axle: mechanical trail, and the stub's collar
                                    # passes behind the barrel so both bearings load from outboard
KP_X = FA_X + KP_TRAIL
KP_BOSS = 12.0                      # arm bosses round the kingpin: tire inner face stays 8 mm off its axis
KNUCKLE_Z = (20.6, KNUCKLE_Z_TOP)   # 27.4 mm barrel (bottom: 5 mm droop + 0.6 off the lower arm) on a brass bush (Rev E: 14 mm, PETG on steel)
BUMP_MM, DROOP_MM = 7.0, 5.0        # travel the checks prove clear; bump_stop sleeve enforces 7
MAX_LOCK_DEG = 28.0                 # radio endpoint; the sweep check proves it clears
STEER_ARM = 28.0                    # kingpin to knuckle ball, straight back (solved: 99% Ackermann)
KINGPIN_Y = TRACK / 2 - 21.0        # 21 mm scrub: housing, race shim and clamp hex (front_wheel_centre)
STEER_ARM_OUT = 0.0                 # ball sits this x STEER_ARM inboard(-)/outboard(+) of straight back
KP_Z0, KP_Z1 = 9.0, 69.0            # hardened 3 mm rod, cut 60 mm
LOWER_Z = (TOP, TOP + 6.0)          # sits on the pan top
UPPER_Z = (64.0, 69.0)              # 16 mm over the barrel: 7 mm bump + a 9 mm bump-stop sleeve
FE_X0, FE_X1 = 112.5, 158.0         # front-end base: starts 0.5 ahead of the splice block
FE_BOLTS_X = (118.0, 152.0)
FOAM_HW = 55.0                      # foam bumper half-width, trimmed from the Schumacher block (tire swing)
BUMPER_X = FA_X + R_W + 10.0        # foam's rear face: 10 mm ahead of the tire nose

POD_PLATE_Y, POD_T = 45.0, 5.0      # pod side plates (PETG: motor runs hot)
PIVOT_X, PIVOT_Z = -78.0, 16.0
LUG_Y0, LUG_T = 14.2, 9.4           # pan pivot lugs, 9.4 mm thick
POD_LUG_HW = 13.9                   # pod lug half-width: shoulder (10) ends here, head lands 0.3 past the pan lug,
                                    # and the lugs sit 0.3 apart: 0.3 mm float each way, never clamps
SHOCK_X, SHOCK_Y = -104.0, 34.0     # two Injora 53 mm coilovers, one each side (Rev E: one spring, off-centre)
SHOCK_Z0, SHOCK_L = 18.0, 46.0     # bottom eye height; eye-to-eye at ride height (39 to 53 available)
BODY_POST_X_R = -122.0             # rear body posts on arms off the pan's shock towers (the deck, not the pod)
POD_BRACE_X = RA_X + 8              # brace sits between the plates, ahead of the axle

BATT = (138.0, 47.0, 25.0)          # 2S hard case, VERIFY the pack you buy
BATT_X0 = -70.0                     # 2 mm off the pivot lugs; leaves an 8 mm lead gap ahead of the servo block


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
        y0, y1 = sorted((s * LUG_Y0, s * (LUG_Y0 + LUG_T)))
        p += bx(PAN_X0, PIVOT_X + 6, y0, y1, TOP, PIVOT_Z + 6)
        p -= cy(PIVOT_X, PIVOT_Z, y0 - 1, y1 + 1, M4_SHOULDER)
    # shock towers: posts outboard of the pivot bolt heads, arms back over the pod to the top eyes
    zt = SHOCK_Z0 + SHOCK_L
    for s in (1, -1):
        y0, y1 = sorted((s * (SHOCK_Y - 4), s * (SHOCK_Y + 4)))
        p += bx(PAN_X0, -80, y0, y1, TOP, zt + 4)
        ya, yb = sorted((s * (SHOCK_Y - 3), s * (SHOCK_Y + 3)))
        p += bx(SHOCK_X + 1.6, -80, ya, yb, zt - 4, zt + 4)                       # arm: ear beside the top eye
        p -= cx(s * SHOCK_Y, zt, SHOCK_X + 1.6 - 0.01, SHOCK_X + 1.6 + M3_INS_DEPTH, M3_INS)
        ya, yb = sorted((s * (BODY_POST_Y_R - 4), s * (BODY_POST_Y_R + 4)))
        p += bx(PAN_X0, -80, *sorted((s * (SHOCK_Y - 4), s * (BODY_POST_Y_R + 4))), zt - 4, zt + 4)   # tower top
        p += bx(BODY_POST_X_R - 5, -80, ya, yb, zt - 4, zt + 4)                   # post arm over the pod
        zr = post_base(BODY_POST_X_R, s * BODY_POST_Y_R, zt + 4)
        if zr > zt + 4.5:
            p += cz(BODY_POST_X_R, s * BODY_POST_Y_R, zt, zr, 9.0)                # riser to the standoff
        p -= insert(BODY_POST_X_R, s * BODY_POST_Y_R, zr)
    # steering servo sinks through the pan: a 1 mm skin is left under it
    sv = bought.LP_SERVO
    p -= bx(SERVO_X - sv["w"] / 2 - 0.3, SERVO_X + sv["w"] / 2 + 0.3, sv["spline_off"] - sv["l"] - 0.3,
            sv["spline_off"] + 0.3, SERVO_Z0, TOP + 1)
    p -= bx(SERVO_X - 4, SERVO_X + 4, sv["spline_off"] - sv["l"] - 12, sv["spline_off"] - sv["l"], SERVO_Z0, TOP + 1)
    # front-end bolt pattern (inserts live in the front end, bolts come up from below)
    for x in FE_BOLTS_X:
        for y in (-30, 30):
            p -= cz(x, y, FLOOR_Z - 1, TOP + 1, M3_CLR)
            p -= cz(x, y, FLOOR_Z - 1, FLOOR_Z + 1.8, 6.0)    # button-head counterbore
    # splice block bolts either side of the joint
    for x, y in SPLICE_PTS:
        p -= cz(x, y, FLOOR_Z - 1, TOP + 1, M3_CLR)
        p -= cz(x, y, FLOOR_Z - 1, FLOOR_Z + 1.8, 6.0)
    # battery strap slots (20 mm velcro) and ESC / RX tie-down slots
    for x in (-40, 40):
        p -= bx(x - 1.5, x + 1.5, 4, 46, FLOOR_Z - 1, TOP + 1)
    for x in (-30, -5, 20, 45):
        p -= bx(x - 1.5, x + 1.5, -46, -30, FLOOR_Z - 1, TOP + 1)
    # front corners narrowed: the 164 mm track swings the tires inside the old 52 mm half-width at lock
    for s in (1, -1):
        p -= bx(FA_X - 40, PAN_X1 + 1, *sorted((s * PAN_FRONT_HW, s * (HALF_W + 1))), FLOOR_Z - 1, TOP + RAIL_H + 1)
    # chamfer the nose so it climbs kerbs instead of digging in
    p -= Pos(PAN_X1, 0, FLOOR_Z) * Rot(0, 35, 0) * Box(12, 2 * HALF_W + 2, 6)
    return p


def pan_halves():
    full = pan()
    rear = full & bx(-300, JOINT_X, -60, 60, 0, 100)                 # keep the shock towers
    front = full & bx(JOINT_X, PAN_X1 + 5, -60, 60, 0, 100)
    return rear, front


SPLICE_PTS = [(JOINT_X - 7, y) for y in (-44, -36, 22, 40)] + [(JOINT_X + 7, y) for y in (-46, 25, 44)]


def splice_servo_block():
    """Bridges the pan joint AND carries the low-profile steering servo, whose case drops through
    the pan to a 1 mm skin. The joint (x 86) is whole under the block; the servo sits ahead of it."""
    sv = bought.LP_SERVO
    b = bx(JOINT_X - 12, SERVO_X + sv["w"] / 2 + 4, -HALF_W + RAIL_T, HALF_W - RAIL_T, TOP, TOP + 8)
    for x, y in SPLICE_PTS:
        b -= insert(x, y, TOP, down=False)
    y1 = sv["spline_off"]                                             # spline on the centreline
    y0 = y1 - sv["l"]
    b -= bx(SERVO_X - sv["w"] / 2 - 0.3, SERVO_X + sv["w"] / 2 + 0.3, y0 - 0.3, y1 + 0.3, TOP - 1, TOP + 9)
    ymid = (y0 + y1) / 2
    ear_z = SERVO_Z0 + sv["ear_z"]
    for s in (1, -1):                                                 # ear posts rise to the ear underside
        py = ymid + s * sv["hole_spc"] / 2
        pa, pb = sorted((py - s * 4.0, py + s * 5.0))
        post = bx(SERVO_X - 7, SERVO_X + 7, pa, pb, TOP, ear_z)
        post -= insert(SERVO_X, py, ear_z)
        b += post
    b -= bx(SERVO_X - 4, SERVO_X + 4, y0 - 12, y0, TOP - 1, TOP + 9)   # lead tunnel under the -Y ear
    for s in (1, -1):                                                 # same narrowed corners as the pan
        b -= bx(FA_X - 40, FE_X0, *sorted((s * PAN_FRONT_HW, s * HALF_W)), TOP - 1, TOP + 30)
    return b


# ---------------------------------------------------------------- body
BODY_POST_X_F, BODY_POST_Y_F, BODY_POST_Y_R = 154.0, 20.0, 42.0
BODY_CLIP = 9.0                     # post stands this far above the body skin: R-clip hole at +4


@lru_cache(maxsize=1)
def body():
    """Venturi GT shell (bought, clear polycarbonate) with 7 mm post holes. (shell, windows, wing)."""
    shell, windows, wing = bought.body_shell(FA_X, RA_X, R_W)
    for x, y in ((BODY_POST_X_F, BODY_POST_Y_F), (BODY_POST_X_F, -BODY_POST_Y_F),
                 (BODY_POST_X_R, BODY_POST_Y_R), (BODY_POST_X_R, -BODY_POST_Y_R)):
        shell -= cz(x, y, 0, 200, 7.0)
    return shell, windows, wing


def post_base(x, y, seat):
    """Where the standoff screws in: a printed riser makes up a 58 mm standoff that is too short;
    one that is too long is cut down (aluminium, hacksaw), so the base stays on the seat."""
    return max(seat, body_post_top(x, y) - bought.POST_STANDOFF)


@lru_cache(maxsize=None)
def body_post_top(x, y):
    shell, windows, _ = bought.body_shell(FA_X, RA_X, R_W)
    return round(bought.body_inner_z(shell + windows, x, y) + BODY_CLIP, 1)


# ---------------------------------------------------------------- front end
def front_end():
    """Base, centre tower, both lower arms and the foam-bumper plate, one print. Hardened 3 mm
    kingpins press into the lower-arm bosses. Rev F: the kingpin sits KP_TRAIL ahead of the
    axle (mechanical trail, self-centring) and 21 mm in from the wheel centre (was 31.5)."""
    f = bx(FE_X0, FE_X1, -40, 40, TOP, TOP + 8)                     # 8 mm: 6.3 mm inserts leave a 1.7 mm cap
    f += bx(KP_X - 9, KP_X + 9, -30, 30, TOP, UPPER_Z[0])          # tower
    for s in (1, -1):
        y0, y1 = sorted((s * 30, s * KINGPIN_Y))
        arm = bx(KP_X - 9, KP_X + 9, y0, y1, LOWER_Z[0], LOWER_Z[1])
        arm += cz(KP_X, s * KINGPIN_Y, LOWER_Z[0], LOWER_Z[1], KP_BOSS)
        arm -= cz(KP_X, s * KINGPIN_Y, LOWER_Z[0] - 1, LOWER_Z[1] + 1, ROD3_PRESS)
        f += arm
        for x in (KP_X - 5, KP_X + 5):                                 # upper-arm inserts on the tower top
            f -= insert(x, s * 22, UPPER_Z[0])
    for x in FE_BOLTS_X:
        for y in (-30, 30):
            f -= insert(x, y, TOP, down=False)
    # bumper plate: carries the foam block clear ahead of the tires (tire nose at FA_X + R_W)
    f += bx(FE_X1, BUMPER_X, -FOAM_HW, FOAM_HW, FLOOR_Z, FLOOR_Z + 5)
    f += bx(FE_X1 - 6, FE_X1, -40, 40, FLOOR_Z, TOP + 8)
    for y in (-FOAM_HW + 12, 0, FOAM_HW - 12):                         # foam is glued, plus 3 zip-tie slots
        f -= bx(BUMPER_X - 8, BUMPER_X - 5, y - 4, y + 4, FLOOR_Z - 1, FLOOR_Z + 6)
    # body-post risers: the bought 58 mm standoffs screw into inserts on top
    for y in (-BODY_POST_Y_F, BODY_POST_Y_F):
        z = post_base(BODY_POST_X_F, y, TOP + 8)
        if z > TOP + 8.5:
            f += cz(BODY_POST_X_F, y, TOP + 7, z, 9.0)
        f -= insert(BODY_POST_X_F, y, z)
    return f


def upper_arm(side=1):
    s = side
    y0, y1 = s * 17, s * KINGPIN_Y
    a = bx(KP_X - 9, KP_X + 9, min(y0, y1), max(y0, y1), *UPPER_Z)
    a += cz(KP_X, s * KINGPIN_Y, *UPPER_Z, KP_BOSS)
    a -= cz(KP_X, s * KINGPIN_Y, UPPER_Z[0] - 1, UPPER_Z[1] + 1, ROD3_PRESS)
    for x in (KP_X - 5, KP_X + 5):
        a -= cz(x, s * 22, UPPER_Z[0] - 1, UPPER_Z[1] + 1, M3_CLR)
    return a


H_LEN = 14.0                        # knuckle housing: 1 lip + 5 bearing + 3 spacer + 5 bearing
BUSH_OD = 4.0                       # K&S 9822 brass tube 4.0 x 3.1, pressed into the barrel
BARREL_D = BUSH_OD + 5.0


def knuckle(side=1):
    """Turns and slides on a brass bush over the hardened kingpin. Both 685ZZ and the 3 mm
    inner-race spacer go in from the OUTBOARD end against a 1 mm lip (Rev E's inner seat was
    blocked on both sides). The barrel sits KP_TRAIL ahead of the axle, so the stub's inboard
    collar passes behind it. Steering arm 8 mm deep, gusseted, ball stud on top."""
    import math
    s = side
    kz0, kz1 = KNUCKLE_Z
    ky = s * KINGPIN_Y
    y_in, y_out = sorted((ky, s * (KINGPIN_Y + H_LEN)))
    k = cz(KP_X, ky, kz0, kz1, BARREL_D)
    k += cy(FA_X, AXLE_Z, y_in, y_out, 16)
    k += bx(FA_X, KP_X, *sorted((ky, s * (KINGPIN_Y + H_LEN - 2))), AXLE_Z - 6, AXLE_Z + 6)   # web
    k += bx(FA_X + 3, KP_X, *sorted((ky + s * 0.5, ky + s * 3.0)), kz0, kz1)                   # barrel web, outboard of the collar
    arm_x, arm_y = arm_tip(s)
    ln = math.dist((arm_x, arm_y), (KP_X, ky))
    ang = math.degrees(math.atan2(arm_y - ky, arm_x - KP_X))
    # dog-leg: a high run (8 x 8) from the barrel passes 1 mm over the stub's spinning collar, then
    # drops at the tip, whose stud seat keeps the ball level with the steering bar's
    def along(d):                                                    # point d mm from the tip toward the kingpin
        return arm_x + (KP_X - arm_x) * d / ln, arm_y + (ky - arm_y) * d / ln
    hi0 = AXLE_Z + 5 + 1.0                                           # collar top (Ø10) + 1 mm
    lo0 = ARM_TIP_TOP - 8
    cx_, cy_ = along((ln + 7.0) / 2)
    k += Pos(cx_, cy_, hi0 + 4) * Rot(0, 0, ang) * Box(ln - 7.0, 8, 8)            # high run, clear of the cup
    d0 = 7.0                                                         # drop starts clear of the ball cup
    d1 = ln - (KP_TRAIL + 5 + 1.0)                                   # and ends 1 mm short of the collar
    jx, jy = along((d0 + d1) / 2)
    k += Pos(jx, jy, (lo0 + hi0 + 8) / 2) * Rot(0, 0, ang) * Box(d1 - d0, 8, hi0 + 8 - lo0)   # drop
    k += cz(arm_x, arm_y, lo0, ARM_TIP_TOP, 9)
    k -= insert(arm_x, arm_y, ARM_TIP_TOP)                           # ball stud screws in here
    k -= cz(KP_X, ky, kz0 - 1, kz1 + 1, BUSH_OD + HOLE_COMP)         # brass bush press
    lip_in, lip_out = sorted((ky, ky + s * 1.0))
    k -= cy(FA_X, AXLE_Z, lip_in - 0.01, lip_out + 0.01, 9.0)        # lip: outer race stops, inner race free
    seat = sorted((ky + s * 1.0, s * (KINGPIN_Y + H_LEN) + s * 1))
    k -= cy(FA_X, AXLE_Z, seat[0], seat[1], BRG_OD)
    k -= cy(FA_X, AXLE_Z, y_in - 1, y_out + 1, 5.6)
    return k


def _rot(x, y, a):
    import math
    return x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)


def arm_tip(s, p=0.0):
    """Knuckle ball-stud centre (x, y) for side s at wheel angle p (rad, + = turning left)."""
    vx, vy = _rot(-STEER_ARM, s * STEER_ARM_OUT * STEER_ARM, p)
    return KP_X + vx, s * KINGPIN_Y + vy


def horn_ball(s, t=0.0):
    """Steering-bar ball for side s at servo angle t: the bar rides the servo saver's arm."""
    vx, vy = _rot(BALL_R, s * BAR_Y, t)
    return SERVO_X + vx, vy


def steering():
    """Planar linkage solve: servo saver + bar (two ball studs) -> tie rod -> knuckle arm.
    Each wheel is tracked from straight-ahead in small steps so the solver stays on the physical
    branch. Inner/outer are named by the turn direction. Returns, for BOTH turn directions, the
    servo angle for MAX_LOCK on the inner wheel, Ackermann % and the smallest transmission angle;
    plus bump steer and rod length."""
    import math
    hz = BALL_Z_KN - _ball_z_bar()               # static height between the ball pairs (3D rod length)
    L = {s: math.sqrt(math.dist(horn_ball(s), arm_tip(s)) ** 2 + hz * hz) for s in (1, -1)}

    def track(s, t, dz=0.0, steps=200):
        p = 0.0
        for i in range(1, steps + 1):
            h = horn_ball(s, t * i / steps)
            f = lambda q: math.dist(h, arm_tip(s, q)) ** 2 + (hz + dz) ** 2 - L[s] ** 2
            for _ in range(40):
                d = (f(p + 1e-7) - f(p - 1e-7)) / 2e-7
                if abs(d) < 1e-12:
                    raise ValueError("linkage toggles")
                p -= f(p) / d
            if abs(p) > 1.2:
                raise ValueError("linkage runs away")
        h, a, k = horn_ball(s, t), arm_tip(s, p), (KP_X, s * KINGPIN_Y)
        u, w = (a[0] - h[0], a[1] - h[1]), (a[0] - k[0], a[1] - k[1])
        ta = math.degrees(math.acos(abs(u[0] * w[0] + u[1] * w[1]) / (math.hypot(*u) * math.hypot(*w))))
        return p, ta

    out = {"_lock_poses": []}                    # (servo angle rad, left wheel rad, right wheel rad)
    for sign in (1, -1):
        lock, d = math.radians(MAX_LOCK_DEG), 0.0
        while True:
            d += 0.1
            t = sign * math.radians(d)
            (pl, tl), (pr, tr) = track(1, t, steps=60), track(-1, t, steps=60)
            if (pl > 0) != (pr > 0):
                raise ValueError(f"wheels steer opposite ways at servo {d:.1f} deg")
            left_turn = pl > 0
            inner, outer = (abs(pl), abs(pr)) if left_turn else (abs(pr), abs(pl))
            if inner >= lock or d > 70:
                break
        out["_lock_poses"].append((t, pl, pr))
        ideal = math.atan(1 / (1 / math.tan(inner) + 2 * KINGPIN_Y / WB))
        out["left" if left_turn else "right"] = {
            "servo_deg": round(d, 1), "inner_deg": round(math.degrees(inner), 1),
            "outer_deg": round(math.degrees(outer), 1), "ideal_outer_deg": round(math.degrees(ideal), 1),
            "ackermann_pct": round(100 * (inner - outer) / (inner - ideal)),
            "min_transmission_deg": round(min(tl, tr))}
    out["bump_steer_deg_per_3mm"] = round(math.degrees(track(1, 0.0, 3.0, steps=1)[0]), 2)
    out["bump_steer_deg_full"] = round(max(abs(math.degrees(track(1, 0.0, dz, steps=1)[0]))
                                         for dz in (BUMP_MM, -DROOP_MM)), 2)
    out["rod_mm"] = round(L[1], 1)
    return out


def steer_bar():
    """Printed bar on the servo saver's arm: carries both tie-rod ball studs. One M3 through the
    saver arm's hole, and a channel over the arm stops it rotating."""
    s = bought.SAVER
    z_top = SERVO_Z0 + bought.LP_SERVO["h"] + s["h"]
    b = bx(SERVO_X + SAVER_R - 5, SERVO_X + BALL_R + 5, -BAR_Y - 5, BAR_Y + 5, z_top, z_top + 8)
    b += bx(SERVO_X + s["hub_d"] / 2 + 0.5, SERVO_X + SAVER_R - 2, -s["arm_w"] / 2 - 2.5, -s["arm_w"] / 2 - 0.2,
            z_top - s["arm_t"], z_top)
    b += bx(SERVO_X + s["hub_d"] / 2 + 0.5, SERVO_X + SAVER_R - 2, s["arm_w"] / 2 + 0.2, s["arm_w"] / 2 + 2.5,
            z_top - s["arm_t"], z_top)
    b -= cz(SERVO_X + SAVER_R, 0, z_top - 1, z_top + 5, M3_CLR)
    for k in (1, -1):                                                # ball studs screw up into the bar
        b -= insert(SERVO_X + BALL_R, k * BAR_Y, z_top, down=False)
    return b


# ---------------------------------------------------------------- rear pod
BRG_LIP = 1.0                       # inboard lip on the pod-plate bearing seats
MESH_TRAVEL = 1.0                   # motor slides +-1.0 mm: 13T needs -0.26, 17T +0.80 from the 14T centre
RAIL_Z = 16.0                       # floor rails Z 12..20, plate bolts go in sideways at this height
RAIL_X = {1: (MOTOR_X - 20, RA_X + 18), -1: (MOTOR_X + 21, RA_X + 18)}   # -Y rail stops short of the motor can
RAIL_BOLTS = {1: (MOTOR_X - 12, RA_X - 8, RA_X + 12), -1: (MOTOR_X + 26, RA_X + 12)}


def pod_plate(side=1):
    """Side plate: one 685ZZ for the axle. The right (-Y) plate also mounts the motor."""
    s = side
    y0, y1 = sorted((s * POD_PLATE_Y, s * (POD_PLATE_Y + POD_T)))
    p = bx(MOTOR_X - 20, RA_X + 18, y0, y1, 12, 52)                  # sits on the floor
    # 685 seat pressed from outboard against a 1 mm inboard lip (Rev F: the old through-bore
    # held every lateral load on the press fit alone)
    yl0, yl1 = sorted((s * POD_PLATE_Y, s * (POD_PLATE_Y - BRG_LIP)))
    p += cy(RA_X, AXLE_Z, yl0, yl1, 14)                                # 1.5 mm off the motor can at 13T
    p -= cy(RA_X, AXLE_Z, yl0 - 1, yl1 + 1, 9.0)
    p -= cy(RA_X, AXLE_Z, y0, y1 + 1, BRG_OD) if s > 0 else cy(RA_X, AXLE_Z, y0 - 1, y1, BRG_OD)
    if s < 0:
        # motor boss clearance + two M3 slots: sliding the motor along X sets mesh
        for dx in (-MESH_TRAVEL, MESH_TRAVEL):                        # boss slot: 13T to 17T all mesh
            p -= cy(MOTOR_X + dx, AXLE_Z, y0 - 1, y1 + 1, 13.5)
        p -= bx(MOTOR_X - MESH_TRAVEL, MOTOR_X + MESH_TRAVEL, y0 - 1, y1 + 1, AXLE_Z - 6.75, AXLE_Z + 6.75)
        for dz in (MOTOR_HOLE_SPC / 2, -MOTOR_HOLE_SPC / 2):
            for dx in (-MESH_TRAVEL - 0.5, 0, MESH_TRAVEL + 0.5):
                p -= cy(MOTOR_X + dx, AXLE_Z + dz, y0 - 1, y1 + 1, M3_CLR)
    else:
        p -= cy(MOTOR_X, AXLE_Z, y0 - 1, y1 + 1, 24)                 # lightening
    p -= cy(POD_BRACE_X + 5, 48, y0 - 1, y1 + 1, M3_CLR)              # brace bolt
    if s < 0:                                                         # gear cover inserts, outer face
        for x, z in ((MOTOR_X - 4, AXLE_Z + 16), (RA_X + 14, AXLE_Z + 20)):
            p -= cy(x, z, y0 - 0.01, y0 + M3_INS_DEPTH - 1.5, M3_INS)
    # side bolts into the floor rail inserts
    for x in RAIL_BOLTS[s]:
        p -= cy(x, RAIL_Z, y0 - 1, y1 + 1, M3_CLR)
    return p


def pod_floor():
    f = bx(MOTOR_X - 20, PAN_X0 - 2.5, -POD_PLATE_Y - POD_T, POD_PLATE_Y + POD_T, 8, 12)
    # motor can and spur clearance
    f -= bx(MOTOR_X - 19, MOTOR_X + 19, -POD_PLATE_Y, 6, 11, 13)
    for s in (1, -1):                                                # rails the plates bolt to sideways
        ya, yb = sorted((s * (POD_PLATE_Y - 8), s * POD_PLATE_Y))
        f += bx(*RAIL_X[s], ya, yb, 12, 20)
        for x in RAIL_BOLTS[s]:
            yi0, yi1 = sorted((s * POD_PLATE_Y + s * 0.01, s * (POD_PLATE_Y - M3_INS_DEPTH)))
            f -= cy(x, RAIL_Z, yi0, yi1, M3_INS)
    # pivot lug: rides between the pan lugs on the M4 shoulder bolt
    # Each uxcell M4 shoulder bolt (5 x 10 mm shoulder) passes through a 10 mm pan lug
    # and threads into an M4 heat-set insert here, so the shoulder is the bearing.
    f += bx(PAN_X0 - 2.5, PIVOT_X + 6, -POD_LUG_HW, POD_LUG_HW, TOP + 1.8, PIVOT_Z + 6)  # clears the pan through +-4 deg
    f += bx(PAN_X0 - 8, PAN_X0 - 2.5, -POD_LUG_HW, POD_LUG_HW, 8, PIVOT_Z + 6)   # full-height web: the lug
    # used to hang on a 1.2 mm neck where it met the floor (Rev E review P0)
    for s in (1, -1):
        y0, y1 = sorted((s * (POD_LUG_HW + 0.1), s * 6.0))
        f -= cy(PIVOT_X, PIVOT_Z, y0, y1, 5.6)                        # M4 insert pilot
    # shock bottom ears, beside each eye, M3 along X into an insert
    for s in (1, -1):
        ya, yb = sorted((s * (SHOCK_Y - 3), s * (SHOCK_Y + 3)))
        f += bx(SHOCK_X + 1.6, SHOCK_X + 9.6, ya, yb, 12, SHOCK_Z0 + bought.SHOCK["eye_d"] / 2 - 0.5)   # under the body
        f -= cx(s * SHOCK_Y, SHOCK_Z0, SHOCK_X + 1.6 - 0.01, SHOCK_X + 1.6 + M3_INS_DEPTH, M3_INS)
    # rear top brace ties the two plates so the pod does not lozenge
    return f


def pod_brace():
    bx0 = POD_BRACE_X
    b = bx(bx0, bx0 + 10, -POD_PLATE_Y, POD_PLATE_Y, 44, 52)
    for s in (1, -1):                                                # side inserts, bolts through the plates
        y0, y1 = sorted((s * POD_PLATE_Y, s * (POD_PLATE_Y - M3_INS_DEPTH)))
        b -= cy(bx0 + 5, 48, y0 - 0.01, y1, M3_INS)
    return b


def battery_stop():
    """Forward stop for the pack (Rev F): a 250 g pack at 30-50 g in a crash otherwise lands on its
    own leads and the XT60. Bolts down through the pan with 2 M3; the slot passes the leads."""
    x0 = BATT_X0 + BATT[0] + 0.5
    s = bx(x0, x0 + 4, 3, 46, TOP, TOP + 18)
    s -= bx(x0 - 1, x0 + 5, 17, 35, TOP + 6, TOP + 19)                     # lead slot
    for y in (8, 41):
        s += bx(x0, x0 + 4, y - 4, y + 4, TOP, TOP + 18)
        s -= insert(x0 + 2, y, TOP, down=False)
    return s


GC_WALL = 1.6


def gear_cover():
    """Spur and pinion guard (Rev F): closed sides and top, open underneath so grit falls out.
    Two M3 into inserts in the motor plate's outer face; the coupling hub passes a 14 mm hole."""
    y_in, y_out = -POD_PLATE_Y - POD_T - 0.3, SPUR_FACE - bought.FLANGE["flange_t"] - 0.5 - GC_WALL
    rs = SPUR_D / 2 + 3
    x0, x1 = MOTOR_X - 9, RA_X + rs
    z1 = AXLE_Z + rs
    z_open = AXLE_Z - SPUR_D / 2 + 2                                          # sides come down past the teeth
    outer = bx(x0, x1, y_out, y_in, z_open, z1) + cy(RA_X, AXLE_Z, y_out, y_in, 2 * rs)
    inner = (bx(x0 + GC_WALL, x1 - GC_WALL, y_out + GC_WALL, y_in + 1, z_open - 1, z1 - GC_WALL) +
             cy(RA_X, AXLE_Z, y_out + GC_WALL, y_in + 1, 2 * rs - 2 * GC_WALL))
    c = outer - inner
    c -= bx(x0 - 1, x1 + 1, y_out - 1, y_in + 1, -10, z_open + 0.01)          # open underneath
    c -= cy(RA_X, AXLE_Z, y_out - 1, y_out + GC_WALL + 1, 14)                 # coupling hub passes
    for x, z in ((MOTOR_X - 4, AXLE_Z + 16), (RA_X + 14, AXLE_Z + 20)):      # tabs to the plate, inboard of
        c += bx(x - 4, x + 4, SPUR_FACE + SPUR_W + 0.8, y_in, z - 4, z + 4)   # the spur's face
        c -= cy(x, z, y_out - 1, y_in + 1, M3_CLR)
    return c


AXLE_FLAT = 2.0                     # ground flat for the coupling's set screws (0.5 mm deep)
SPUR_W = 6.0                        # VERIFY: thickness of the spur you buy at the hub
SPUR_FACE = -POD_PLATE_Y - POD_T - 11.0  # outboard face of the spur, against the coupling flange
CLAMP_T = 3.5
PINION_W, PINION_HUB = 5.0, 4.0     # 48P pinion: tooth face, set-screw hub
PINION_FACE = SPUR_FACE + 0.5       # outboard tooth face; the shaft (15 mm exposed) must reach it


def _bolt_circle(r=None, n=4):
    import math
    r = r if r is not None else bought.FLANGE["bc"] / 2
    return [(RA_X + r * math.cos(math.radians(45 + a)), AXLE_Z + r * math.sin(math.radians(45 + a)))
            for a in range(0, 360, 360 // n)]


def spur_carrier():
    """Rev F replaces the printed D-bore spool. Torque goes axle flat -> the aluminium flange
    coupling's two set screws -> its flange -> four M3 through the spur -> this ring. The ring's
    spigot centres the spur (drilled 10.2) on the axle, its 7 mm tip bears on the right bearing's
    inner race, and its nut pockets take the four M3s. Also the drilling template for the spur."""
    from build123d import RegularPolygon, extrude
    y0, y1 = SPUR_FACE + SPUR_W, SPUR_FACE + SPUR_W + CLAMP_T
    y_tip = -POD_PLATE_Y - POD_T
    c = cy(RA_X, AXLE_Z, y0, y1, 26) + cy(RA_X, AXLE_Z, y1, y_tip, 7)
    c += cy(RA_X, AXLE_Z, y0 - SPUR_W + 0.5, y0, 10.0)               # spigot into the spur's centre
    c -= cy(RA_X, AXLE_Z, y0 - SPUR_W, y_tip + 1, 5.0 + 0.15 + HOLE_COMP)
    for x, z in _bolt_circle():
        c -= cy(x, z, y0 - 1, y1 + 1, M3_CLR)
        nut = extrude(RegularPolygon(6.4 / 2, 6), 2.6)
        c -= Pos(x, y1 + 0.01, z) * Rot(90, 0, 0) * nut
    return c


def race_shim():
    """5 x 7 x 1 mm: lets a collar or hub bear on a 685's inner race only. Two go between the front
    hub collars and the outer knuckle bearings, one between the rear-left collar and its bearing."""
    return cz(0, 0, 0, 1.0, 6.5) - cz(0, 0, -1, 2, 5.0 + 0.3 + HOLE_COMP)   # 6.5: clear of the 685 shield


def lip_shim():
    """5.4 x 6.5 x 1.3: inside each knuckle's lip, between the stub's inboard collar and the inner
    bearing's inner race, so the turning collar never rubs the lip."""
    return cz(0, 0, 0, 1.3, 6.5) - cz(0, 0, -1, 2, 5.0 + 0.3 + HOLE_COMP)


def race_spacer():
    """3 mm inner-race spacer between the two knuckle bearings (the stack clamps through it)."""
    return cz(0, 0, 0, 3.0, 6.5) - cz(0, 0, -1, 4, 5.0 + 0.3 + HOLE_COMP)


def bump_stop():
    """Sleeve on the kingpin inside the spring: knuckle hits it after BUMP_MM of travel.
    Needs a spring with ID over 5.4 mm. Modelled at the origin, print 2."""
    gap = UPPER_Z[0] - KNUCKLE_Z[1]
    return cz(0, 0, 0, gap - BUMP_MM, 5.0) - cz(0, 0, -1, gap, ROD3_SLIDE)


# ---------------------------------------------------------------- electronics trays
ESP_C = (34.0, -30.0)               # PCB centre; USB end faces -X (rear), antenna overhang +X
ESP_LEDGE_Z = TOP + 2 + 9.5         # header pins hang 8.5 mm under the PCB: leads are soldered, not Duponts
ESP_WALL, ESP_CLR = 1.6, 0.3


def _esp_inner():
    L, W, _ = bought.DEVKIT
    return (ESP_C[0] - L / 2 - 1.0 - ESP_CLR, ESP_C[0] + L / 2 + bought.DEVKIT_OVERHANG + ESP_CLR,
            ESP_C[1] - W / 2 - ESP_CLR, ESP_C[1] + W / 2 + ESP_CLR)


def _esp_bosses():
    x0, x1, y0, y1 = _esp_inner()
    return [(x0 - ESP_WALL - 1.9, ESP_C[1] + 9), (x1 + ESP_WALL + 1.9, ESP_C[1] - 9)]   # off each end: the
    # long sides sit against the pan rail and the battery


ESP_TOP = ESP_LEDGE_Z + 1.6 + 3.1 + 0.6                                # PCB + shield + 0.6 air


def esp32_cradle():
    """Well for an ESP32-DevKitC V4. The board has no mounting holes: it rests on two ledges
    between the header rows and the lid's pad holds the module shield 0.2 mm above, so the
    board is captured. The base sticks to the pan with servo tape."""
    x0, x1, y0, y1 = _esp_inner()
    w = ESP_WALL
    L = bought.DEVKIT[0]
    pcb0, pcb1 = ESP_C[0] - L / 2, ESP_C[0] + L / 2
    c = bx(x0 - w, x1 + w, y0 - w, y1 + w, TOP, ESP_TOP)
    c -= bx(x0, x1, y0, y1, TOP + 2, ESP_TOP + 1)
    for xa, xb in ((pcb0 + 0.3, pcb0 + 3.0), (pcb1 - 3.0, pcb1 - 0.3)):   # under the PCB, inside the pin rows
        c += bx(xa, xb, ESP_C[1] - 9, ESP_C[1] + 9, TOP + 2, ESP_LEDGE_Z)
    c -= bx(x0 - w - 1, x0 + 0.1, ESP_C[1] - 5, ESP_C[1] + 5, ESP_LEDGE_Z - 0.5, ESP_TOP + 1)   # USB port
    c -= bx(x0 - w - 1, x0 + 0.1, ESP_C[1] - 5, ESP_C[1] + 5, TOP + 3, TOP + 9)                 # lead exit
    for x, y in _esp_bosses():
        c += cz(x, y, TOP, ESP_TOP, 7) + bx(*sorted((x, x0 - w / 2 if x < x0 else x1 + w / 2)), y - 3.5, y + 3.5, TOP, ESP_TOP)
        c -= insert(x, y, ESP_TOP)
    return c


def esp32_lid():
    """Clamps the board: one pad stops 0.2 mm above the module shield. LED window; the antenna
    end is plastic only (no metal near the PCB antenna)."""
    x0, x1, y0, y1 = _esp_inner()
    w = ESP_WALL
    L = bought.DEVKIT[0]
    mx1 = ESP_C[0] + L / 2 + bought.DEVKIT_OVERHANG
    shield_top = ESP_LEDGE_Z + 1.6 + 3.1
    lid = bx(x0 - w, x1 + w, y0 - w, y1 + w, ESP_TOP, ESP_TOP + 1.6)
    lid += bx(mx1 - 24.0, mx1 - 8.0, ESP_C[1] - 6, ESP_C[1] + 6, shield_top + 0.2, ESP_TOP)
    for x, y in _esp_bosses():
        lid += cz(x, y, ESP_TOP, ESP_TOP + 1.6, 7)
        lid -= cz(x, y, ESP_TOP - 1, ESP_TOP + 3, M3_CLR)
    lid -= cz(ESP_C[0] - L / 2 + 10, ESP_C[1] + 8, ESP_TOP - 1, ESP_TOP + 3, 3.0)   # LED window
    return lid


# ---------------------------------------------------------------- reference bodies (not printed)
def wheel(x, y):
    return cy(x, AXLE_Z, y - W_W / 2, y + W_W / 2, 2 * R_W)


def _wheel_at(x, s, centre_y):
    tire, rim = bought.wheel()
    place = Pos(x, centre_y, AXLE_Z) * Rot(-90 if s > 0 else 90, 0, 0)   # outer face outboard
    return place * tire, place * rim


def front_wheel_centre():
    """Stack-up outward from the kingpin line: housing (lip, bearing, spacer, bearing), race shim,
    the clamping hex's flange, then the hex seats fully in the rim pocket, whose inboard end is
    RIM_BOSS - RIM_OFFSET inboard of the rim centre."""
    return KINGPIN_Y + H_LEN + 1.0 + bought.CLAMP_HEX["flange_t"] + (bought.RIM_BOSS - bought.RIM_OFFSET)


BALL_Z_KN = ARM_TIP_TOP + 4.8       # stud up from the knuckle arm tip (cup clears the seat)
FRONT_STUB = 0.0                    # set in refs: the stub axle's inboard end


def _ball_z_bar():
    return SERVO_Z0 + bought.LP_SERVO["h"] + bought.SAVER["h"] - 6.3   # stud down from the bar, 1.5 spacer


def tie_rods(t=0.0, pl=0.0, pr=0.0, dz=0.0):
    """Both bought tie rods (cups + cut M3 rod + balls) at servo angle t and wheel angles pl, pr."""
    cups, rods, balls = [], [], []
    for s, p in ((1, pl), (-1, pr)):
        hx, hy = horn_ball(s, t)
        ax, ay = arm_tip(s, p)
        c, rd, b = bought.tie_rod((hx, hy, _ball_z_bar()), (ax, ay, BALL_Z_KN + dz))
        cups.append(c)
        rods.append(rd)
        balls.append(b)
    return Compound(cups), Compound(rods), Compound(balls)


def refs():
    """Bought parts, detailed from published dimensions (see bought.py), in the assembled pose."""
    import math
    r = {}
    tires, rims, hexes = [], [], []
    hx = bought.clamp_hex()
    for x in (FA_X, RA_X):
        for s in (1, -1):
            tr, rm = _wheel_at(x, s, s * TRACK / 2)
            tires.append(tr)
            rims.append(rm)
            y_fl = s * (TRACK / 2 - bought.CLAMP_HEX["flange_t"] - (bought.RIM_BOSS - bought.RIM_OFFSET))
            hexes.append(Pos(x, y_fl, AXLE_Z) * Rot(-90 if s > 0 else 90, 0, 0) * hx)
    r["tires"], r["rims"], r["clamp_hexes"] = Compound(tires), Compound(rims), Compound(hexes)
    r["battery"] = Pos(BATT_X0, 1.2, TOP) * bought.battery()
    r["motor"] = Pos(MOTOR_X, -POD_PLATE_Y, AXLE_Z) * Rot(-90, 0, 0) * bought.motor()
    r["spur"] = Pos(RA_X, SPUR_FACE, AXLE_Z) * Rot(-90, 0, 0) * bought.gear(SPUR_T, DP, SPUR_W, 10.2)
    # teeth outboard over the spur's face, hub inboard against the plate (Rev E had 1 mm of face overlap)
    r["pinion"] = (Pos(MOTOR_X, PINION_FACE, AXLE_Z) * Rot(-90, 0, 0) *
                   bought.gear(PINION_T, DP, PINION_W, 3.17, hub_d=6.5, hub_l=PINION_HUB))
    r["coupling"] = Pos(RA_X, SPUR_FACE, AXLE_Z) * Rot(90, 0, 0) * bought.flange_coupling()
    sv = bought.LP_SERVO
    r["servo"] = Pos(SERVO_X, sv["spline_off"], SERVO_Z0) * bought.lp_servo()
    r["servo_saver"] = Pos(SERVO_X, 0, SERVO_Z0 + sv["h"]) * bought.servo_saver()
    r["tie_rod_cups"], r["tie_rods"], r["ball_studs"] = tie_rods()
    r["esc"] = Pos(ESC_X0, ESC_Y0, TOP) * bought.esc()
    board, pins = bought.esp32_devkit()
    r["esp32"] = Pos(ESP_C[0], ESP_C[1], ESP_LEDGE_Z) * board
    r["esp32_pins"] = Pos(ESP_C[0], ESP_C[1], ESP_LEDGE_Z) * pins
    r["rear_axle"] = (cy(RA_X, AXLE_Z, -TRACK / 2 - 4, TRACK / 2 + 4, 5) -
                      bx(RA_X - 3, RA_X + 3, -60, -48, AXLE_Z + AXLE_FLAT, AXLE_Z + 3))   # flat under the set screws
    # front stubs: inboard collar behind the barrel, through both bearings, out through the clamp hex
    stubs, collars = [], []
    for s in (1, -1):
        y_in = s * (KINGPIN_Y - 0.3)
        stubs.append(cy(FA_X, AXLE_Z, *sorted((y_in - s * 5.0, s * (TRACK / 2 + 4))), 5))
        collars.append(cy(FA_X, AXLE_Z, *sorted((y_in - s * 5.0, y_in)), 10) -
                       cy(FA_X, AXLE_Z, *sorted((y_in - s * 6.0, y_in + s)), 5.05))
    r["front_axles"] = Compound(stubs)
    collar = lambda y0, y1: cy(RA_X, AXLE_Z, y0, y1, 10) - cy(RA_X, AXLE_Z, y0 - 1, y1 + 1, 5.05)
    collars.append(collar(POD_PLATE_Y + POD_T + 1.0, POD_PLATE_Y + POD_T + 6.0))   # rear left, outboard
    r["collars"] = Compound(collars)
    r["kingpins"] = Compound([cz(KP_X, s * KINGPIN_Y, KP_Z0, KP_Z1, 3) for s in (1, -1)])
    r["bushes"] = Compound([cz(KP_X, s * KINGPIN_Y, KNUCKLE_Z[0], KNUCKLE_Z[1], BUSH_OD) -
                            cz(KP_X, s * KINGPIN_Y, 0, 90, 3.1) for s in (1, -1)])
    r["shocks"] = Compound([Pos(SHOCK_X, s * SHOCK_Y, SHOCK_Z0) * bought.shock(SHOCK_L) for s in (1, -1)])
    r["foam_bumper"] = Pos(BUMPER_X, 0, FLOOR_Z + 5) * bought.foam_bumper()
    posts = []
    seats = {BODY_POST_X_F: TOP + 8, BODY_POST_X_R: SHOCK_Z0 + SHOCK_L + 4}
    for x, y in ((BODY_POST_X_F, BODY_POST_Y_F), (BODY_POST_X_F, -BODY_POST_Y_F),
                 (BODY_POST_X_R, BODY_POST_Y_R), (BODY_POST_X_R, -BODY_POST_Y_R)):
        z = post_base(x, y, seats[x])
        posts.append(Pos(x, y, z) * bought.body_standoff(body_post_top(x, y) - z))
    r["body_posts"] = Compound(posts)
    r["body"], r["windows"], r["wing"] = body()
    return r


ESC_X0, ESC_Y0 = -38.0, -44.0       # QuicRun 10BL60 G2, fan up; motor leads leave -X, battery leads +X


def leads():
    """Indicative 14 AWG routes (3.0 mm OD silicone). Flexible, so not in the overlap check."""
    ex, ey, ez = ESC_X0, ESC_Y0 + bought.ESC[1] / 2, TOP + 8
    motor_end = (MOTOR_X, -POD_PLATE_Y + bought.MOTOR["l"] + 6, AXLE_Z)
    out = []
    for i, dy in enumerate((-4, 0, 4)):                                # motor A B C: slack loop over the pivot
        out.append(bought.tube([(ex, ey + dy, ez), (-95, -30 + dy, 24), (-118, -2 + dy, 40),
                                (motor_end[0] + 8 * (i - 1), motor_end[1] + 4, 40),
                                (motor_end[0] + 8 * (i - 1), motor_end[1], AXLE_Z + 6)], 3.0))
    bx1 = BATT_X0 + BATT[0]
    for dz, y in ((0, 30), (4, 22)):                                     # pack to ESC, between pack and servo block
        out.append(bought.tube([(bx1, y, TOP + 12 + dz), (bx1 + 3, y, TOP + 12 + dz), (bx1 + 3, -8, TOP + 12 + dz),
                                (2, -8, TOP + 12 + dz), (ex + bought.ESC[0] + 3, ey, TOP + 8 + dz),
                                (ex + bought.ESC[0], ey, TOP + 8 + dz)], 3.0))
    xt60 = bx(30, 46, -12, -4, TOP + 7, TOP + 21)                        # XT60 pair in the run
    return Compound(out + [xt60])


# ---------------------------------------------------------------- build
def parts():
    rear, front = pan_halves()
    return {
        "pan_rear": (rear, "PETG, flat, 5 walls, 40% gyroid"),
        "pan_front": (front, "PETG, flat, 5 walls, 40% gyroid"),
        "splice_servo_block": (splice_servo_block(), "PETG, flat"),
        "front_end": (front_end(), "PETG, base down, 6 walls, 60% infill"),
        "upper_arm_L": (upper_arm(1), "PETG, flat, 100% infill"),
        "upper_arm_R": (upper_arm(-1), "PETG, flat, 100% infill"),
        "knuckle_L": (knuckle(1), "PETG, barrel upright, 100% infill"),
        "knuckle_R": (knuckle(-1), "PETG, barrel upright, 100% infill"),
        "steer_bar": (steer_bar(), "PETG, flat, 100% infill"),
        "pod_plate_L": (pod_plate(1), "PETG, flat, 100% infill"),
        "pod_plate_R_motor": (pod_plate(-1), "PETG, flat, 100% infill (motor heat)"),
        "pod_floor": (pod_floor(), "PETG, flat"),
        "pod_brace": (pod_brace(), "PETG, flat"),
        "spur_carrier": (spur_carrier(), "PETG, flat, nut pockets up, 100% infill"),
        "gear_cover": (gear_cover(), "PETG, outer face down"),
        "battery_stop": (battery_stop(), "PETG, flat"),
        "esp32_cradle": (esp32_cradle(), "PETG, base down"),
        "esp32_lid": (esp32_lid(), "PETG, top face down"),
        "race_shims_x3": (Pos(60, -150, 0) * race_shim(), "PETG, flat, print 3"),
        "lip_shims_x2": (Pos(75, -150, 0) * lip_shim(), "PETG, flat, print 2"),
        "race_spacers_x2": (Pos(90, -150, 0) * race_spacer(), "PETG, flat, print 2"),
        "bump_stops": (Compound([Pos(KP_X, s * KINGPIN_Y, KNUCKLE_Z[1]) * bump_stop() for s in (1, -1)]),
                       "PETG, upright, print 2"),
    }


def checks(ps, r):
    """Fit checks that fail loudly if a knob change breaks the car."""
    def gap(a, b):
        return a.distance_to(b)
    solid = {k: v[0] for k, v in ps.items()}
    assert gap(r["motor"], r["rear_axle"]) > 1, "motor can hits the rear axle"
    assert AXLE_Z - SPUR_D / 2 > 6, f"spur ground clearance {AXLE_Z - SPUR_D / 2:.1f} mm"
    assert AXLE_Z - MOTOR_D / 2 > 10, "motor can too low"
    assert gap(r["motor"], solid["pod_floor"]) > 0.5, "motor hits pod floor"
    assert gap(r["battery"], solid["splice_servo_block"]) > 0.5, "battery hits servo block"
    assert gap(r["servo"], r["battery"]) > 0.5, "servo hits battery"
    assert gap(r["foam_bumper"], r["tires"]) > 8, "foam bumper does not stand ahead of the tires"
    assert r["foam_bumper"].bounding_box().max.X > r["tires"].bounding_box().max.X + 15, "bumper too short"
    for k, lim in (("pan_rear", 3), ("pan_front", 3), ("front_end", 1.5)):   # front_end: the kingpin bosses are
        assert gap(r["tires"], solid[k]) > lim, f"wheel rubs {k}"           # coaxial with steering, so 2 mm holds at any lock
    assert gap(r["spur"], solid["pod_plate_R_motor"]) > 0.5, "spur rubs motor plate"
    assert gap(r["esc"], r["battery"]) > 0.5, "esc hits battery"
    # every printed part fits the Ender 3 V3 bed in at least one orientation
    for k, (s, _) in ps.items():
        bb = sorted([s.bounding_box().size.X, s.bounding_box().size.Y, s.bounding_box().size.Z])
        assert bb[1] <= 215 and bb[2] <= 245, f"{k} does not fit a 220x220x250 bed {bb}"
        assert s.is_valid, f"{k} invalid solid"
    st = steering()
    poses = st.pop("_lock_poses")
    for turn in ("left", "right"):
        assert turn in st, f"never reached {turn} lock {st}"
        assert 70 <= st[turn]["ackermann_pct"] <= 130, f"Ackermann {st}"
        assert 25 <= st[turn]["servo_deg"] <= 45, f"servo throw for lock {st}"
        assert st[turn]["min_transmission_deg"] >= 40, f"linkage near toggle {st}"
    assert st["left"]["servo_deg"] == st["right"]["servo_deg"], f"asymmetric linkage {st}"
    assert abs(st["bump_steer_deg_per_3mm"]) < 0.5, f"bump steer {st}"
    assert st["bump_steer_deg_full"] < 1.6, f"bump steer at full travel {st}"
    # tie rod buckling: cut M3 stainless rod at its thread root (2.39 mm), pinned-pinned over the
    # full ball-to-ball length (the stiff cups ignored, so conservative), vs the servo's stall push
    i_min = 3.1416 * 2.39 ** 4 / 64
    p_cr = 3.1416 ** 2 * 193000 * i_min / st["rod_mm"] ** 2
    f_stall = bought.LP_SERVO["stall_nm"] / BALL_R * 1000
    st["tie_rod_buckling_sf"] = round(p_cr / f_stall, 1)
    assert p_cr / f_stall >= 2.0, f"tie rod buckles under servo stall: P_cr {p_cr:.0f} N vs {f_stall:.0f} N"
    # nothing overlaps anything in the assembled pose: printed parts AND the bought parts
    # (axle, kingpins, collars, motor, spur, pinion, servo, ESC, battery, wheels)
    # gear mesh: pinion teeth sit across the spur's face, the shaft reaches them, the hub clears the plate
    p_teeth = (PINION_FACE, PINION_FACE + PINION_W)
    overlap = min(p_teeth[1], SPUR_FACE + SPUR_W) - max(p_teeth[0], SPUR_FACE)
    assert overlap >= 0.9 * PINION_W, f"pinion face overlap {overlap:.1f} of {PINION_W} mm"
    assert -POD_PLATE_Y - bought.MOTOR["shaft_l"] <= PINION_FACE + 0.5, "motor shaft does not reach the pinion face"
    assert PINION_FACE + PINION_W + PINION_HUB <= -POD_PLATE_Y - POD_T - 0.4, "pinion hub hits the motor plate"
    # front wheel stack-up: knuckle bearing, race shim, hub collar and rim hex put the wheel where TRACK says
    assert abs(front_wheel_centre() - TRACK / 2) < 0.5, f"front wheel centre {front_wheel_centre():.1f} vs {TRACK / 2}"
    # body posts reach through the shell, clip hole above the skin
    for x, y in ((BODY_POST_X_F, BODY_POST_Y_F), (BODY_POST_X_R, BODY_POST_Y_R)):
        assert body_post_top(x, y) - BODY_CLIP > TOP + 30, f"body skin implausibly low at {x}, {y}"
    # knuckle bearings load from outboard: an 11 mm disc slides from the open end to both seats
    for s, kn in ((1, "knuckle_L"), (-1, "knuckle_R")):
        path = cy(FA_X, AXLE_Z, *sorted((s * (KINGPIN_Y + 1.05), s * (KINGPIN_Y + H_LEN + 20))), 10.9)
        v = (solid[kn] & path).volume
        assert v < 0.01, f"{kn}: bearings cannot go in from outboard ({v:.1f} mm3 in the way)"
        lip = cy(FA_X, AXLE_Z, *sorted((s * KINGPIN_Y, s * (KINGPIN_Y + 1))), 10.9) - cy(FA_X, AXLE_Z, -200, 200, 9.0)
        assert (solid[kn] & lip).volume > 0.9 * lip.volume, f"{kn}: no lip behind the inner bearing"
    assert bought.CLAMP_HEX["hex_t"] <= bought.HEX_DEPTH + 0.01, "clamp hex longer than the rim's hex pocket"
    # every bought part's real envelope: ball cups, balls, saver, shocks, standoffs, hexes all included
    tiny = ("race_shims_x3", "lip_shims_x2", "race_spacers_x2")
    allb = {k: v for k, v in solid.items() if k not in tiny}
    allb.update({"ref_" + k: v for k, v in r.items()})
    touching = [{"ref_spur", "ref_pinion"},                          # meshing teeth touch by design; CENTRE sets them
                {"ref_tie_rod_cups", "ref_ball_studs"}, {"ref_tie_rod_cups", "ref_tie_rods"},   # ball joints
                {"ref_body", "ref_body_posts"}]                     # posts pass 7 mm holes; clipped above
    names = list(allb)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if {a, b} in touching:
                continue
            v = (allb[a] & allb[b]).volume
            assert v < 0.01, f"{a} overlaps {b} by {v:.2f} mm3"
    # steering sweep: wheel + knuckle turn about each kingpin to full lock both ways
    fixed = [solid[k] for k in ("front_end", "upper_arm_L", "upper_arm_R", "pan_front",
                                "splice_servo_block")] + [r["body"], r["foam_bumper"]]
    worst = 99.0
    for s, kn in ((1, "knuckle_L"), (-1, "knuckle_R")):
        whl = wheel(FA_X, s * TRACK / 2)
        for deg in (-MAX_LOCK_DEG, -MAX_LOCK_DEG / 2, MAX_LOCK_DEG / 2, MAX_LOCK_DEG):
            ax = Axis((KP_X, s * KINGPIN_Y, 0), (0, 0, 1))
            for moving in (whl, solid[kn]):
                m = moving.rotate(ax, deg)
                for f in fixed:
                    worst = min(worst, m.distance_to(f))
    assert worst > 0.8, f"steering sweep clearance {worst:.2f} mm at <= {MAX_LOCK_DEG} deg"
    # tie rods at both locks: the bought cups and rods vs chassis, servo, saver, the other knuckle, wheels
    import math
    rod_gap = 99.0
    for t, pl, pr in poses:
        wheels = [wheel(FA_X, s * TRACK / 2).rotate(Axis((KP_X, s * KINGPIN_Y, 0), (0, 0, 1)), math.degrees(p))
                  for s, p in ((1, pl), (-1, pr))]
        kns = [solid[kn].rotate(Axis((KP_X, s * KINGPIN_Y, 0), (0, 0, 1)), math.degrees(p))
               for s, kn, p in ((1, "knuckle_L", pl), (-1, "knuckle_R", pr))]
        cups, rods, _ = tie_rods(t, pl, pr)
        # each cup is bolted to its own ball; trim 6 mm round both balls of each rod for the test
        trim = None
        for s, p in ((1, pl), (-1, pr)):
            for (x, y) in (horn_ball(s, t), arm_tip(s, p)):
                c = cz(x, y, 0, 100, 12)
                trim = c if trim is None else trim + c
        body = (cups + rods) - trim
        saver = r["servo_saver"].rotate(Axis((SERVO_X, 0, 0), (0, 0, 1)), math.degrees(t))   # turns with the servo
        for f in fixed + [r["servo"], saver] + kns + wheels:
            rod_gap = min(rod_gap, body.distance_to(f))
    assert rod_gap > 1.0, f"tie rod clearance at lock {rod_gap:.2f} mm"
    # suspension travel: knuckle + wheel slide on the kingpin; also bump at full lock
    travel = 99.0
    for s, kn in ((1, "knuckle_L"), (-1, "knuckle_R")):
        ax = Axis((KP_X, s * KINGPIN_Y, 0), (0, 0, 1))
        for dz in (BUMP_MM, -DROOP_MM):
            for deg in (0, MAX_LOCK_DEG, -MAX_LOCK_DEG):
                for moving in (wheel(FA_X, s * TRACK / 2), solid[kn]):
                    m = Pos(0, 0, dz) * moving.rotate(ax, deg)
                    for f in fixed:
                        travel = min(travel, m.distance_to(f))
    assert travel > 0.5, f"front travel clearance {travel:.2f} mm"
    # rear pod swings about the pivot: +-4 deg is about +-7 mm at the axle
    pod = [solid[k] for k in ("pod_floor", "pod_plate_L", "pod_plate_R_motor", "pod_brace", "spur_carrier",
                              "gear_cover")]
    pod += [r["motor"], r["spur"], r["coupling"], r["tires"], r["rims"]]
    pax = Axis((PIVOT_X, 0, PIVOT_Z), (0, 1, 0))
    lugs = [bx(PAN_X0, PIVOT_X + 6, *sorted((s * LUG_Y0, s * (LUG_Y0 + LUG_T))), TOP, PIVOT_Z + 6) for s in (1, -1)]
    pan_nolug = solid["pan_rear"] - lugs[0] - lugs[1]
    # the shock length at either stop must stay inside the shock's travel
    for a in (-4, 4):
        for s in (1, -1):
            top = (SHOCK_X, s * SHOCK_Y, SHOCK_Z0 + SHOCK_L)
            ra = math.radians(a)                                     # bottom eye rides the pod about the pivot (Y axis)
            dx, dz = SHOCK_X - PIVOT_X, SHOCK_Z0 - PIVOT_Z
            bot = (PIVOT_X + dx * math.cos(ra) + dz * math.sin(ra), s * SHOCK_Y, PIVOT_Z - dx * math.sin(ra) + dz * math.cos(ra))
            ln = math.dist(top, bot)
            assert bought.SHOCK["comp"] + 0.5 <= ln <= bought.SHOCK["ext"] - 0.5, f"shock at {a} deg: {ln:.1f} mm"
    pod_gap = min(p.rotate(pax, a).distance_to(pan_nolug) for p in pod for a in (-4, 4))
    assert pod_gap > 0.3, f"pod swing clearance {pod_gap:.2f} mm"
    import math
    drop = (PIVOT_X - RA_X) * math.sin(math.radians(4))                # pivot drop at 4 deg of pod swing
    edge = drop * (FA_X - PAN_X0) / (FA_X - PIVOT_X)                   # chassis pitches about the front axle
    assert FLOOR_Z - edge > 1.0, f"pan rear edge touches the ground at 4 deg of pod swing ({FLOOR_Z - edge:.1f} mm)"
    assert LUG_Y0 - POD_LUG_HW >= 0.25, "pivot lugs have no side float"
    lc = load_cases()
    st["loads"] = lc
    assert lc["kingpin_kerb_N"] >= 150, f"kingpin too weak for a kerb tap {lc}"
    assert lc["steer_arm_sf"] >= 1.5, f"knuckle steering arm too weak at servo stall {lc}"
    assert lc["spool_torque_sf"] >= 3, f"spur drive too weak {lc}"
    assert lc["pod_web_sf"] >= 3, f"pod lug web too weak {lc}"
    print(f"checks ok: clearances, spur/motor height, bed fit, validity, no part overlaps, "
          f"steering sweep +-{MAX_LOCK_DEG} deg min gap {worst:.1f} mm, front travel +{BUMP_MM}/-{DROOP_MM} "
          f"min gap {travel:.1f} mm, pod swing +-4 deg min gap {pod_gap:.1f} mm "
          f"tie rods at lock min gap {rod_gap:.1f} mm, steering {st} "
          f"front wheel centre {front_wheel_centre():.1f} mm, body posts F {body_post_top(BODY_POST_X_F, BODY_POST_Y_F)} R {body_post_top(BODY_POST_X_R, BODY_POST_Y_R)} "
          f"(NOT tested: unpublished servo/saver/hex/shock dims (VERIFY), tie rods during bump, lead routes, "
          f"fatigue, interlayer strength beyond the in-plane allowables, impact above 15 km/h)")


# allowables, deliberately low: PETG in-layer 40 MPa (printed, warm), hardened chrome rod 1000 MPa
# (HRC 58-62 rod yields well above this), M4 set screw shear 500 MPa
PETG_ALLOW, PIN_ALLOW, SCREW_SHEAR = 40.0, 1000.0, 500.0


def load_cases():
    """Closed-form strength checks for the parts the Rev E review found weak. Each returns a
    capacity or safety factor; checks() asserts the floors."""
    import math
    out = {}
    # kingpin: hardened 3 mm, simply supported between the arm bosses, lateral load mid-span
    span = UPPER_Z[0] - LOWER_Z[1]
    z_sec = math.pi * 3.0 ** 3 / 32
    out["kingpin_kerb_N"] = round(PIN_ALLOW * z_sec * 4 / span)
    # knuckle steering arm: servo stall through the bar's ball radius, bending at the barrel, 8 x 8
    f_rod = bought.LP_SERVO["stall_nm"] / BALL_R * 1000
    sigma = 6 * f_rod * STEER_ARM / (8 * 8 ** 2)
    out["steer_arm_sf"] = round(PETG_ALLOW / sigma, 2)
    # spur drive: peak axle torque (60 A, Kt 2.65 mNm/A, 6.21:1) through two M4 set screws on the
    # 2 mm flat, each in shear at the flat's lever arm
    t_axle = 60 * 60 / (2 * math.pi * 3600) * (SPUR_T / PINION_T) * 1000
    t_cap = 2 * SCREW_SHEAR * math.pi * 3.1 ** 2 / 4 * AXLE_FLAT
    out["spool_torque_sf"] = round(t_cap / t_axle, 1)
    # pod lug web: 10 g on the rear (0.55 x 1.33 kg x 10 x 9.81) as a vertical load at the axle,
    # bending the 27.8 x 14 web over the axle-to-pivot lever
    f_v = 0.55 * 1.33 * 9.81 * 10       # review/data.json cg: 1.33 kg, 55% rear
    m = f_v * (PIVOT_X - RA_X)
    w_sec = 2 * POD_LUG_HW * (PIVOT_Z + 6 - 8) ** 2 / 6
    out["pod_web_sf"] = round(PETG_ALLOW / (m / w_sec), 1)
    return out


# rotation (deg about X, then Y, then Z) that puts each part in its slicer orientation
PRINT_ROT = {
    # knuckles print as modelled: barrel upright (round kingpin bore), arm flat in the layers,
    # housing on its side (the 11.2 seat prints horizontally, which the coupon calibrates)
    "pod_plate_L": (90, 0, 0), "pod_plate_R_motor": (-90, 0, 0),   # lay the plates flat
    "spur_carrier": (-90, 0, 0),                                   # nut pockets up
    "gear_cover": (-90, 0, 0),                                     # outer face down
}


def print_pose(shape, rot):
    s = shape
    for ax, a in zip((Axis.X, Axis.Y, Axis.Z), rot):
        if a:
            s = s.rotate(ax, a)
    bb = s.bounding_box()
    return Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * s


def calibration_coupon():
    """Print this first (PETG, same settings as the bearing parts) and measure every hole
    with the caliper. Left to right on top: 685 seat 11.2, kingpin press 3.0, brass-bush press 4.2,
    carrier bore 5.35, M3 insert 4.2, M4 insert 5.6, M3 clearance 3.4, bearing lip 9.0. On the side
    face: 11.2 and 4.2 printed horizontally (the knuckle housing prints on its side).
    Adjust HOLE_COMP until the 685 drops in with a light press."""
    holes = [11.2, 3.0, BUSH_OD + HOLE_COMP, 5.35, 4.2, 5.6, 3.4, 9.0]
    xs, x = [], 4.0
    for d in holes:
        x += d / 2 + 3
        xs.append(x)
        x += d / 2
    w = x + 4
    c = bx(0, w, 0, 18, 0, 8)
    for xc, d in zip(xs, holes):
        c -= cz(xc, 9, -1, 9, d)
    c += bx(0, 30, 18, 32, 0, 16)                                    # side block for horizontal bores
    c -= cy(9, 8, 17, 33, 11.2)
    c -= cy(23, 8, 17, 33, BUSH_OD + HOLE_COMP)
    return c


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    (OUT / "print").mkdir(exist_ok=True)
    ps, r = parts(), refs()
    checks(ps, r)
    import json
    manifest = {}
    for name, (shape, note) in ps.items():
        export_stl(shape, str(OUT / f"{name}.stl"))
        export_step(shape, str(OUT / f"{name}.step"))
        pp = print_pose(shape, PRINT_ROT.get(name, (0, 0, 0)))
        assert pp.bounding_box().size.X <= 215 and pp.bounding_box().size.Y <= 215, f"{name} print pose off the bed"
        export_stl(pp, str(OUT / "print" / f"{name}.stl"))
        export_step(pp, str(OUT / "print" / f"{name}.step"))
        bb = shape.bounding_box().size
        manifest[name] = {"print": note, "volume_cm3": round(shape.volume / 1000, 1),
                          "bbox_mm": [round(bb.X, 1), round(bb.Y, 1), round(bb.Z, 1)]}
    coupon = calibration_coupon()
    export_stl(coupon, str(OUT / "print" / "calibration_coupon.stl"))
    export_step(coupon, str(OUT / "print" / "calibration_coupon.step"))
    asm = Compound([s for k, (s, _) in ps.items() if "_x" not in k] + list(r.values()) + [leads()])  # loose small parts left out
    export_step(asm, str(OUT / "wk10_assembly.step"))
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=1))
    print(json.dumps(manifest, indent=1))

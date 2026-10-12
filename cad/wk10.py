"""WK-10: printable 1/10 on-road pan car built around parts already on the shelf.

Run:  C:/Users/bman0/Code/OpsBrain/.venv/Scripts/python.exe C:/Users/bman0/Code/rc-car/cad/wk10.py
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

# steering: MG996R (VERIFY body 40.7 x 19.7 x 42.9, ears 54.5 wide, holes 49.5 x 10)
SERVO = (19.7, 40.7, 42.9)          # X, Y, Z as mounted (long axis across the car)
SERVO_EAR_Z, SERVO_HOLE_SPC = 26.5, 49.5
SERVO_X = 86.0
SPLINE_OFF = 10.2                   # spline centre from the near end of the case

KINGPIN_Y = TRACK / 2 - 31.5        # inboard of the bearing housing (31.5 mm to the wheel centre)
KNUCKLE_Z = (22.0, 36.0)            # barrel on the kingpin: spring gap above, droop room below
BUMP_MM, DROOP_MM = 7.0, 5.0        # travel the checks prove clear; bump_stop sleeve enforces 7
MAX_LOCK_DEG = 28.0                 # firmware endpoint; the sweep check proves it clears
STEER_ARM = 16.0                    # kingpin to tie-rod eye (re-solved for the 164 mm track)
STEER_ARM_OUT = 0.3                 # eye sits 0.3 x STEER_ARM outboard of straight-back (about 17 deg)
HORN_R = 14.0                       # horn hole used, pointing FORWARD; drill the stock horn hole to 3.0 mm
HORN_TOP_Z = 56.0                   # VERIFY: top face of the horn on the mounted servo
EYE_T = 3.0                         # tie-rod eye thickness; both rods stack on one horn bolt
BAR_H = 5.0                         # tie-rod bar depth away from the eyes (Euler buckling vs servo stall)
SERVO_STALL_NM = 1.08               # MG996R listing at 6 V (11 kg-cm); VERIFY, it varies by clone
EYE_Z = {1: HORN_TOP_Z + 1.6, -1: HORN_TOP_Z + 1.6 + EYE_T + 0.2}   # left rod low, right rod high
KP_Z0, KP_Z1 = 9.0, 56.0            # 3 mm rod, cut 47 mm
LOWER_Z = (TOP, TOP + 6.0)     # sits on the pan top
UPPER_Z = (50.0, 55.0)              # 14 mm spring gap; housing clears the arm through bump

POD_PLATE_Y, POD_T = 45.0, 5.0      # pod side plates (PETG: motor runs hot)
PIVOT_X, PIVOT_Z = -78.0, 16.0
LUG_Y0, LUG_T = 14.2, 9.4           # pan pivot lugs, 9.4 mm thick
POD_LUG_HW = 13.9                   # pod lug half-width: shoulder (10) ends here, head lands 0.3 past the pan lug,
                                    # and the lugs sit 0.3 apart: 0.3 mm float each way, never clamps
SPRING_X, SPRING_Y = -102.0, 34.0   # rear pod spring: clear of the brace, the pivot bolt head and the battery
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
    # rear spring brace: post outboard of the +Y pivot bolt head, arm back over the pod floor
    p += bx(PAN_X0, -80, SPRING_Y - 4, SPRING_Y + 4, TOP, 44)
    p += bx(SPRING_X - 6, -80, SPRING_Y - 4, SPRING_Y + 4, 40, 44)
    p -= cz(SPRING_X, SPRING_Y, 39, 45, M3_CLR)
    # front-end bolt pattern (inserts live in the front end, bolts come up from below)
    for x in (112, 140):
        for y in (-30, 30):
            p -= cz(x, y, FLOOR_Z - 1, TOP + 1, M3_CLR)
            p -= cz(x, y, FLOOR_Z - 1, FLOOR_Z + 1.8, 6.0)    # button-head counterbore
    # splice block bolts either side of the joint
    for x in (JOINT_X - 7, JOINT_X + 7):
        for y in SPLICE_Y:
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
    rear = full & bx(-300, JOINT_X, -60, 60, 0, 60)                  # keep the spring arm
    front = full & bx(JOINT_X, PAN_X1 + 5, -60, 60, 0, 60)
    return rear, front


SPLICE_Y = (-44, -36, 22, 40)        # clear of the servo pocket (Y -30.8 .. 10.5)


def splice_servo_block():
    """Bridges the pan joint AND carries the steering servo."""
    sx, sy, sz = SERVO
    b = bx(JOINT_X - 12, JOINT_X + 12, -HALF_W + RAIL_T, HALF_W - RAIL_T, TOP, TOP + 8)
    for x in (JOINT_X - 7, JOINT_X + 7):
        for y in SPLICE_Y:
            b -= insert(x, y, TOP, down=False)
    # servo: spline on the centreline, case runs to -Y
    y_near = SPLINE_OFF
    y0, y1 = y_near - sy, y_near
    b -= bx(SERVO_X - sx / 2 - 0.3, SERVO_X + sx / 2 + 0.3, y0 - 0.3, y1 + 0.3, TOP - 1, TOP + 9)
    # ear posts rise to the ear underside
    ymid = (y0 + y1) / 2
    for s in (1, -1):
        py = ymid + s * SERVO_HOLE_SPC / 2
        pa, pb = sorted((py - s * 4.0, py + s * 5.0))                       # thin side faces the case
        post = bx(SERVO_X - 7, SERVO_X + 7, pa, pb, TOP, TOP + SERVO_EAR_Z)
        post -= insert(SERVO_X, py, TOP + SERVO_EAR_Z)
        b += post
    # the servo lead leaves the bottom of the -Y end face, under the ear: tunnel through that post
    b -= bx(SERVO_X - 4, SERVO_X + 4, y0 - 12, y0, TOP - 1, TOP + 9)
    return b


# ---------------------------------------------------------------- body
BODY_POST_X_F, BODY_POST_Y_F, BODY_POST_Y_R = 144.0, 20.0, 25.0
BODY_CLIP = 9.0                     # post stands this far above the body skin: R-clip hole at +4


@lru_cache(maxsize=1)
def body():
    """Venturi GT shell (bought, clear polycarbonate) with 7 mm post holes. (shell, windows, wing)."""
    shell, windows, wing = bought.body_shell(FA_X, RA_X, R_W)
    import math
    for y in (BODY_POST_Y_F, -BODY_POST_Y_F):
        shell -= cz(BODY_POST_X_F, y, 0, 200, 7.0)
    # rear posts ride the pod: at the 4 deg stop their tops move fore-aft, so those holes are slots
    xr = POD_BRACE_X + 5
    swing = (bought.body_inner_z(shell, xr, BODY_POST_Y_R) - PIVOT_Z) * math.sin(math.radians(4))
    for y in (BODY_POST_Y_R, -BODY_POST_Y_R):
        shell -= cz(xr - swing, y, 0, 200, 7.0) + cz(xr + swing, y, 0, 200, 7.0) + bx(xr - swing, xr + swing, y - 3.5, y + 3.5, 0, 200)
    return shell, windows, wing


@lru_cache(maxsize=None)
def body_post_top(x, y):
    shell, _, _ = bought.body_shell(FA_X, RA_X, R_W)
    return round(bought.body_inner_z(shell, x, y) + BODY_CLIP, 1)


# ---------------------------------------------------------------- front end
def front_end():
    """Base + centre tower + both lower arms, one print, kingpins pressed in."""
    f = bx(104, 150, -40, 40, TOP, TOP + 8)                          # 8 mm: 6.3 mm inserts leave a 1.7 mm cap
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
    for y in (-BODY_POST_Y_F, BODY_POST_Y_F):
        top = body_post_top(BODY_POST_X_F, y)
        f += cz(BODY_POST_X_F, y, TOP + 5, top, 6.0)
        f -= cz(BODY_POST_X_F, y, TOP + 5, top + 1, 2.6)
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
    y_in, y_out = sorted((s * (KINGPIN_Y + 4), s * (KINGPIN_Y + 17)))
    k += cy(FA_X, AXLE_Z, y_in, y_out, 16)
    yb0, yb1 = sorted((s * KINGPIN_Y, s * (KINGPIN_Y + 8)))
    k += bx(FA_X - 5, FA_X + 5, yb0, yb1, kz0, kz1)
    # steering arm: back and angled outboard (steering() solves this to ~98% Ackermann
    # with the forward horn), post up to this side's tie-rod eye height
    import math
    arm_x, arm_y = arm_tip(s)
    ln = math.dist((arm_x, arm_y), (FA_X, s * KINGPIN_Y))
    ang = math.degrees(math.atan2(arm_y - s * KINGPIN_Y, arm_x - FA_X))
    k += Pos((arm_x + FA_X) / 2, (arm_y + s * KINGPIN_Y) / 2, kz1 - 2) * Rot(0, 0, ang) * Box(ln, 6, 4)
    top = EYE_Z[s] - 1.6
    k += cz(arm_x, arm_y, kz1 - 4, top, 8)
    k -= insert(arm_x, arm_y, top)                                   # M3 pivot bolt through the eye
    k -= cz(FA_X, s * KINGPIN_Y, kz0 - 1, kz1 + 1, ROD3_SLIDE)
    # bearing seats from each end, 3 mm land between them
    k -= cy(FA_X, AXLE_Z, y_in - 1, y_in + B685[2], BRG_OD) if s > 0 else \
         cy(FA_X, AXLE_Z, y_out - B685[2], y_out + 1, BRG_OD)
    k -= cy(FA_X, AXLE_Z, y_out - B685[2], y_out + 1, BRG_OD) if s > 0 else \
         cy(FA_X, AXLE_Z, y_in - 1, y_in + B685[2], BRG_OD)
    k -= cy(FA_X, AXLE_Z, y_in - 1, y_out + 1, 7.0)
    return k


def _rot(x, y, a):
    import math
    return x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)


def arm_tip(s, p=0.0):
    """Knuckle steering-arm eye (x, y) for side s at wheel angle p (rad, + = turning left)."""
    vx, vy = _rot(-STEER_ARM, s * STEER_ARM_OUT * STEER_ARM, p)
    return FA_X + vx, s * KINGPIN_Y + vy


def steering():
    """Planar linkage solve: servo horn (pointing forward, both rods on one bolt) ->
    tie rod -> knuckle arm. Each wheel is tracked from straight-ahead in small steps so
    the solver stays on the physical branch. Inner/outer are named by the turn direction.
    Returns, for BOTH turn directions, the servo angle for MAX_LOCK on the inner wheel,
    Ackermann % and the smallest transmission angle; plus bump steer and rod length."""
    import math

    def horn(t):
        vx, vy = _rot(HORN_R, 0.0, t)
        return SERVO_X + vx, vy

    L = {s: math.dist(horn(0), arm_tip(s)) for s in (1, -1)}

    def track(s, t, dz=0.0, steps=200):
        p = 0.0
        for i in range(1, steps + 1):
            h = horn(t * i / steps)
            f = lambda q: math.dist(h, arm_tip(s, q)) ** 2 + dz * dz - L[s] ** 2
            for _ in range(40):                  # Newton from the previous step
                d = (f(p + 1e-7) - f(p - 1e-7)) / 2e-7
                if abs(d) < 1e-12:
                    raise ValueError("linkage toggles")
                p -= f(p) / d
            if abs(p) > 1.2:
                raise ValueError("linkage runs away")
        h, a, k = horn(t), arm_tip(s, p), (FA_X, s * KINGPIN_Y)
        u, w = (a[0] - h[0], a[1] - h[1]), (a[0] - k[0], a[1] - k[1])
        ta = math.degrees(math.acos(abs(u[0] * w[0] + u[1] * w[1]) / (math.hypot(*u) * math.hypot(*w))))
        return p, ta

    out = {"_lock_poses": []}                    # (horn xy, left wheel rad, right wheel rad) at each lock
    for sign in (1, -1):                         # both servo directions
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
        out["_lock_poses"].append((horn(t), pl, pr))
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


def tie_rod_half(side=1, end="horn", p0=None, p1=None):
    """Each tie rod = two printed halves joined by a headless M3 stud (cut an M3x20) running
    in an insert in each half; turning the knuckle half 180 deg moves toe by 0.25 mm."""
    import math
    s = side
    p0 = p0 or (SERVO_X + HORN_R, 0.0)
    p1 = p1 or arm_tip(s)
    L = math.dist(p0, p1)
    ang = math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))
    gap = 4.0
    half = L / 2 - gap / 2
    eye = cz(0, 0, -EYE_T / 2, EYE_T / 2, 8.0) - cz(0, 0, -2, 2, 3.6)   # loose: tilts with bump
    bar = bx(3.5, 8.0, -3, 3, -EYE_T / 2, EYE_T / 2)                    # thin by the eye: the rods stack here
    bar += bx(8.0, half - 4, -3, 3, -EYE_T / 2, -EYE_T / 2 + BAR_H)      # deep, flat-bottomed section
    boss = bx(half - 8, half, -3.8, 3.8, -EYE_T / 2, -EYE_T / 2 + 7.6)   # flat bottom: prints without support
    h = eye + bar + boss
    h -= cx(0, -EYE_T / 2 + 3.8, half - M3_INS_DEPTH, half + 0.01, M3_INS)
    h = h.rotate(Axis.Z, ang if end == "horn" else ang + 180)
    x, y = p0 if end == "horn" else p1
    return Pos(x, y, EYE_Z[s]) * h


# ---------------------------------------------------------------- rear pod
RAIL_Z = 16.0                       # floor rails Z 12..20, plate bolts go in sideways at this height
RAIL_X = {1: (MOTOR_X - 20, RA_X + 18), -1: (MOTOR_X + 21, RA_X + 18)}   # -Y rail stops short of the motor can
RAIL_BOLTS = {1: (MOTOR_X - 12, RA_X - 8, RA_X + 12), -1: (MOTOR_X + 26, RA_X + 12)}


def pod_plate(side=1):
    """Side plate: one 685ZZ for the axle. The right (-Y) plate also mounts the motor."""
    s = side
    y0, y1 = sorted((s * POD_PLATE_Y, s * (POD_PLATE_Y + POD_T)))
    p = bx(MOTOR_X - 20, RA_X + 18, y0, y1, 12, 52)                  # sits on the floor
    p -= cy(RA_X, AXLE_Z, y0 - 1, y1 + 1, BRG_OD)
    if s < 0:
        # motor boss clearance + two M3 slots: sliding the motor along X sets mesh
        p -= cy(MOTOR_X, AXLE_Z, y0 - 1, y1 + 1, 13.5)
        for dz in (MOTOR_HOLE_SPC / 2, -MOTOR_HOLE_SPC / 2):
            for dx in (-1.5, 0, 1.5):
                p -= cy(MOTOR_X + dx, AXLE_Z + dz, y0 - 1, y1 + 1, M3_CLR)
    else:
        p -= cy(MOTOR_X, AXLE_Z, y0 - 1, y1 + 1, 24)                 # lightening
    p -= cy(POD_BRACE_X + 5, 48, y0 - 1, y1 + 1, M3_CLR)              # brace bolt
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
    f += bx(PAN_X0 - 8, PAN_X0 - 2.5, -POD_LUG_HW, POD_LUG_HW, 8, TOP + 3)    # tongue to the floor
    for s in (1, -1):
        y0, y1 = sorted((s * (POD_LUG_HW + 0.1), s * 6.0))
        f -= cy(PIVOT_X, PIVOT_Z, y0, y1, 5.6)                        # M4 insert pilot
    # spring seat under the pan's spring arm + insert for the M3x35 spring bolt
    f += cz(SPRING_X, SPRING_Y, 12, 16, 10)                                    # 8 mm under a 6.3 mm insert
    f -= insert(SPRING_X, SPRING_Y, 16)
    # rear top brace ties the two plates so the pod does not lozenge
    return f


def pod_brace():
    bx0 = POD_BRACE_X
    b = bx(bx0, bx0 + 10, -POD_PLATE_Y, POD_PLATE_Y, 44, 52)
    for s in (1, -1):                                                # side inserts, bolts through the plates
        y0, y1 = sorted((s * POD_PLATE_Y, s * (POD_PLATE_Y - M3_INS_DEPTH)))
        b -= cy(bx0 + 5, 48, y0 - 0.01, y1, M3_INS)
    for y in (-BODY_POST_Y_R, BODY_POST_Y_R):                         # rear body posts
        top = body_post_top(bx0 + 5, y)
        b += cz(bx0 + 5, y, 52, top, 6.0)
        b -= cz(bx0 + 5, y, 52, top + 1, 2.6)
    return b


AXLE_FLAT = 2.0                     # ground flat sits 2.0 mm from the axle centre (0.5 mm deep)
D_BORE = 5.0 + 0.15 + HOLE_COMP     # spool D-bore: slides along 100 mm of axle
D_FLAT = AXLE_FLAT + 0.1 + HOLE_COMP / 2
SPUR_W = 6.0                        # VERIFY: thickness of the spur you buy at the hub
SPUR_FACE = -POD_PLATE_Y - POD_T - 11.0  # outboard face of the spur, against the flange
CLAMP_T = 3.5


def _bolt_circle(r=10.0, n=3):
    import math
    return [(RA_X + r * math.cos(math.radians(a)), AXLE_Z + r * math.sin(math.radians(a)))
            for a in range(0, 360, 360 // n)]


def spool():
    """Locked rear hub. The spur (centre drilled to 12.2 mm) is sandwiched between this
    flange and spur_clamp by 3 M3 bolts. The bore is a D that keys on a 0.5 mm flat ground
    along the axle, so torque goes through the flat. Axially the 7 mm tip bears on the right
    bearing's inner race and a shaft collar outboard holds it there."""
    y_hub = SPUR_FACE + SPUR_W + CLAMP_T                             # hub end under the clamp ring
    y_tip = -POD_PLATE_Y - POD_T                                     # right bearing's outer face
    y_out = SPUR_FACE - 3.0                                          # flush with the flange: prints flange-down
    s = cy(RA_X, AXLE_Z, y_out, y_hub, 12)
    s += cy(RA_X, AXLE_Z, y_hub, y_tip, 7)                            # touches the inner race only
    s += cy(RA_X, AXLE_Z, SPUR_FACE - 3, SPUR_FACE, 30)               # flange
    s -= cy(RA_X, AXLE_Z, y_out - 1, y_tip + 1, D_BORE) & bx(RA_X - 4, RA_X + 4, y_out - 1, y_tip + 1,
                                                            AXLE_Z - 4, AXLE_Z + D_FLAT)
    for x, z in _bolt_circle():
        s -= cy(x, z, SPUR_FACE - 4, SPUR_FACE + 1, M3_CLR)
    return s


def spur_clamp():
    """Clamp ring on the inboard side of the spur, nut pockets facing the plate.
    Also the drilling template: tape it to the spur and drill the 3 holes at 3.2 mm."""
    import math
    from build123d import RegularPolygon, extrude
    y0, y1 = SPUR_FACE + SPUR_W, SPUR_FACE + SPUR_W + CLAMP_T
    c = cy(RA_X, AXLE_Z, y0, y1, 30) - cy(RA_X, AXLE_Z, y0 - 1, y1 + 1, 12.4)
    for x, z in _bolt_circle():
        c -= cy(x, z, y0 - 1, y1 + 1, M3_CLR)
        nut = extrude(RegularPolygon(6.4 / 2, 6), 2.6)                 # M3 nut, 5.5 AF + clearance
        c -= Pos(x, y1 + 0.01, z) * Rot(90, 0, 0) * nut
    return c

HUB_COLLAR = 8.5                    # front: race shim + collar + hex puts the wheel centre on TRACK / 2


def hex_hub():
    """Replaces the Hobbypark hubs, which need an M4-threaded axle tip that plain 5 mm rod
    does not have. 12 mm hex into the rim, collar inboard, D-bore + M3 bolt as a set screw
    against the flat (grind the flat with the rotary tool). Modelled at the origin, bore on Z."""
    import math
    from build123d import RegularPolygon, extrude
    hexa = extrude(RegularPolygon(12.0 / math.sqrt(3), 6, major_radius=True), 5.0)  # 12 AF
    h = Pos(0, 0, HUB_COLLAR) * hexa + cz(0, 0, 0, HUB_COLLAR, 16.0)   # collar under the hex
    h -= cz(0, 0, -1, HUB_COLLAR + 6, 5.0 + 0.15 + HOLE_COMP)
    h -= cx(0, 3.5, 0, 9, M3_CLR).rotate(Axis.Z, 90)                   # radial M3 set bolt
    h -= bx(-3.3, 3.3, 3.6, 6.3, -1, 6.4)                              # nut slot (flats up), loads from below
    return h


def race_shim():
    """5 x 7 x 1 mm: lets a collar or hub bear on a 685's inner race only. Two go between the front
    hub collars and the outer knuckle bearings, one between the rear-left collar and its bearing."""
    return cz(0, 0, 0, 1.0, 6.5) - cz(0, 0, -1, 2, 5.0 + 0.3 + HOLE_COMP)   # 6.5: clear of the 685 shield


def spring_shim(h):
    """Stack under the rear pod spring to set ride height (print one each of 1, 2 and 4 mm)."""
    return cz(0, 0, 0, h, 10.0) - cz(0, 0, -1, h + 1, M3_CLR)


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
    """Stack-up from the knuckle outward: outer 685 face, race shim (1), hub collar, then the hex
    seats in the rim's pocket, whose inboard end is RIM_BOSS - RIM_OFFSET inboard of the rim centre."""
    return KINGPIN_Y + 17 + 1.0 + HUB_COLLAR + (bought.RIM_BOSS - bought.RIM_OFFSET)


def refs():
    """Bought parts, detailed from published dimensions (see bought.py), in the assembled pose."""
    r = {}
    tires, rims = [], []
    for x in (FA_X, RA_X):
        for s in (1, -1):
            tr, rm = _wheel_at(x, s, s * TRACK / 2)
            tires.append(tr)
            rims.append(rm)
    r["tires"], r["rims"] = Compound(tires), Compound(rims)
    r["battery"] = Pos(BATT_X0, 1.2, TOP) * bought.battery()
    r["motor"] = Pos(MOTOR_X, -POD_PLATE_Y, AXLE_Z) * Rot(-90, 0, 0) * bought.motor()
    r["spur"] = Pos(RA_X, SPUR_FACE, AXLE_Z) * Rot(-90, 0, 0) * bought.gear(SPUR_T, DP, SPUR_W, 12.2)
    r["pinion"] = (Pos(MOTOR_X, -POD_PLATE_Y - POD_T - 1, AXLE_Z) * Rot(90, 0, 0) *
                   bought.gear(PINION_T, DP, 5.0, 3.17, hub_d=6.5, hub_l=4.5))
    case, horn = bought.servo(HORN_TOP_Z - TOP, SPLINE_OFF, HORN_R)
    r["servo"] = Pos(SERVO_X, SPLINE_OFF, TOP) * case
    r["servo_horn"] = Pos(SERVO_X, SPLINE_OFF, TOP) * horn
    r["esc"] = Pos(ESC_X0, ESC_Y0, TOP) * bought.esc()
    board, pins = bought.esp32_devkit()
    r["esp32"] = Pos(ESP_C[0], ESP_C[1], ESP_LEDGE_Z) * board
    r["esp32_pins"] = Pos(ESP_C[0], ESP_C[1], ESP_LEDGE_Z) * pins
    r["rear_axle"] = (cy(RA_X, AXLE_Z, -TRACK / 2 - 18, TRACK / 2 + 18, 5) -
                      bx(RA_X - 3, RA_X + 3, -TRACK / 2 - 19, TRACK / 2 + 19, AXLE_Z + AXLE_FLAT, AXLE_Z + 3))
    # 5 mm shaft collars: left outboard of the left plate, right outboard of the spool
    y_sp = SPUR_FACE - 3.0
    collar = lambda y0, y1: cy(RA_X, AXLE_Z, y0, y1, 10) - cy(RA_X, AXLE_Z, y0 - 1, y1 + 1, 5.05)
    r["collars"] = Compound([collar(POD_PLATE_Y + POD_T + 1.0, POD_PLATE_Y + POD_T + 6.0), collar(y_sp - 5, y_sp)])
    r["kingpins"] = Compound([cz(FA_X, s * KINGPIN_Y, KP_Z0, KP_Z1, 3) for s in (1, -1)])
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
        "spur_clamp": (spur_clamp(), "PETG, flat, nut pockets up"),
        "esp32_cradle": (esp32_cradle(), "PLA, base down"),
        "esp32_lid": (esp32_lid(), "PLA, top face down"),
        "tie_rod_L_horn": (tie_rod_half(1, "horn"), "PETG, flat, 100% infill"),
        "tie_rod_L_knuckle": (tie_rod_half(1, "knuckle"), "PETG, flat, 100% infill"),
        "tie_rod_R_horn": (tie_rod_half(-1, "horn"), "PETG, flat, 100% infill"),
        "tie_rod_R_knuckle": (tie_rod_half(-1, "knuckle"), "PETG, flat, 100% infill"),
        "hex_hub_x4": (Pos(0, -150, 0) * hex_hub(), "PETG, collar down, 100% infill, print 4"),
        "race_shims_x4": (Pos(60, -150, 0) * race_shim(), "PETG, flat, print 4"),
        "spring_shims_x3": (Compound([Pos(80 + 14 * i, -150, 0) * spring_shim(h) for i, h in enumerate((1, 2, 4))]),
                            "PETG, flat"),
        "bump_stops": (Compound([Pos(FA_X, s * KINGPIN_Y, KNUCKLE_Z[1]) * bump_stop() for s in (1, -1)]),
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
    for k in ("pan_rear", "pan_front", "front_end"):
        assert gap(r["tires"], solid[k]) > 3, f"wheel rubs {k}"
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
    # tie rod buckling (pinned-pinned Euler, PETG E 2100 MPa, weaker bending axis) vs servo stall push
    i_min = min(6 * BAR_H ** 3, BAR_H * 6 ** 3) / 12
    p_cr = 3.1416 ** 2 * 2100 * i_min / st["rod_mm"] ** 2
    f_stall = SERVO_STALL_NM / HORN_R * 1000
    st["tie_rod_buckling_sf"] = round(p_cr / f_stall, 1)
    assert p_cr / f_stall >= 2.0, f"tie rod buckles under servo stall: P_cr {p_cr:.0f} N vs {f_stall:.0f} N"
    # nothing overlaps anything in the assembled pose: printed parts AND the bought parts
    # (axle, kingpins, collars, motor, spur, pinion, servo, ESC, battery, wheels)
    # front wheel stack-up: knuckle bearing, race shim, hub collar and rim hex put the wheel where TRACK says
    assert abs(front_wheel_centre() - TRACK / 2) < 0.5, f"front wheel centre {front_wheel_centre():.1f} vs {TRACK / 2}"
    # body posts reach through the shell, clip hole above the skin
    for x, y in ((BODY_POST_X_F, BODY_POST_Y_F), (POD_BRACE_X + 5, BODY_POST_Y_R)):
        assert body_post_top(x, y) - BODY_CLIP > TOP + 30, f"body skin implausibly low at {x}, {y}"
    allb = {k: v for k, v in solid.items() if k not in ("hex_hub_x4", "race_shims_x4", "spring_shims_x3")}
    allb.update({"ref_" + k: v for k, v in r.items()})
    names = list(allb)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if {a, b} == {"ref_spur", "ref_pinion"}:                 # meshing teeth touch by design; CENTRE sets them
                continue
            v = (allb[a] & allb[b]).volume
            assert v < 0.01, f"{a} overlaps {b} by {v:.2f} mm3"
    # steering sweep: wheel + knuckle turn about each kingpin to full lock both ways
    fixed = [solid[k] for k in ("front_end", "upper_arm_L", "upper_arm_R", "pan_front",
                                "splice_servo_block")] + [r["body"]]
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
    # tie rods at both locks: rod body (eyes trimmed off) vs chassis, servo and both turned wheels
    import math
    rod_gap = 99.0
    for hxy, pl, pr in poses:
        wheels = [wheel(FA_X, s * TRACK / 2).rotate(Axis((FA_X, s * KINGPIN_Y, 0), (0, 0, 1)), math.degrees(p))
                  for s, p in ((1, pl), (-1, pr))]
        for s, p in ((1, pl), (-1, pr)):
            a = arm_tip(s, p)
            other_kn = solid["knuckle_R" if s > 0 else "knuckle_L"].rotate(
                Axis((FA_X, -s * KINGPIN_Y, 0), (0, 0, 1)), math.degrees(pr if s > 0 else pl))
            # the real printed halves (eyes, bar, 7.6 mm joint boss) in this pose; the eyes are
            # bolted to the horn and the post, so the eye ends are trimmed for the clearance test
            halves = [tie_rod_half(s, e, hxy, a) for e in ("horn", "knuckle")]
            body = (halves[0] + halves[1]) - cz(hxy[0], hxy[1], 0, 90, 9) - cz(a[0], a[1], 0, 90, 9)
            for f in fixed + [r["servo"], other_kn] + wheels:
                rod_gap = min(rod_gap, body.distance_to(f))
    assert rod_gap > 1.0, f"tie rod clearance at lock {rod_gap:.2f} mm"
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
    pod += [r["motor"], r["spur"], r["tires"], r["rims"]]
    pax = Axis((PIVOT_X, 0, PIVOT_Z), (0, 1, 0))
    lugs = [bx(PAN_X0, PIVOT_X + 6, *sorted((s * LUG_Y0, s * (LUG_Y0 + LUG_T))), TOP, PIVOT_Z + 6) for s in (1, -1)]
    pan_nolug = solid["pan_rear"] - lugs[0] - lugs[1]
    pod_gap = min(p.rotate(pax, a).distance_to(pan_nolug) for p in pod for a in (-4, 4))
    assert pod_gap > 0.3, f"pod swing clearance {pod_gap:.2f} mm"
    import math
    drop = (PIVOT_X - RA_X) * math.sin(math.radians(4))                # pivot drop at 4 deg of pod swing
    edge = drop * (FA_X - PAN_X0) / (FA_X - PIVOT_X)                   # chassis pitches about the front axle
    assert FLOOR_Z - edge > 1.0, f"pan rear edge touches the ground at 4 deg of pod swing ({FLOOR_Z - edge:.1f} mm)"
    assert LUG_Y0 - POD_LUG_HW >= 0.25, "pivot lugs have no side float"
    print(f"checks ok: clearances, spur/motor height, bed fit, validity, no part overlaps, "
          f"steering sweep +-{MAX_LOCK_DEG} deg min gap {worst:.1f} mm, front travel +{BUMP_MM}/-{DROOP_MM} "
          f"min gap {travel:.1f} mm, pod swing +-4 deg min gap {pod_gap:.1f} mm "
          f"tie rods at lock min gap {rod_gap:.1f} mm, steering {st} "
          f"front wheel centre {front_wheel_centre():.1f} mm, body posts F {body_post_top(BODY_POST_X_F, BODY_POST_Y_F)} R {body_post_top(POD_BRACE_X + 5, BODY_POST_Y_R)} "
          f"(NOT tested: real rim hex depth, spring solid height, tie rods during bump, body vs pod swing, lead routes)")


# rotation (deg about X, then Y, then Z) that puts each part in its slicer orientation
PRINT_ROT = {
    "knuckle_L": (-90, 0, 0), "knuckle_R": (90, 0, 0),             # axle bore vertical, housing end down
    "pod_plate_L": (90, 0, 0), "pod_plate_R_motor": (-90, 0, 0),   # lay the plates flat
    "spool": (90, 0, 0),                                           # flange face down
    "spur_clamp": (90, 0, 0),                                      # nut pockets up
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
    with the caliper. Left to right on top: 685 seat 11.2, kingpin press 3.0, kingpin slide
    3.5, hub bore 5.35, M3 insert 4.2, M4 insert 5.6, M3 clearance 3.4, spool D-bore. On the side face:
    the same 11.2 and 3.5 bores printed horizontally (the knuckle's kingpin bore prints that way).
    Adjust HOLE_COMP until the 685 drops in with a light press."""
    holes = [11.2, 3.0, 3.5, 5.35, 4.2, 5.6, 3.4, D_BORE]
    xs, x = [], 4.0
    for d in holes:
        x += d / 2 + 3
        xs.append(x)
        x += d / 2
    w = x + 4
    c = bx(0, w, 0, 18, 0, 8)
    for xc, d in zip(xs, holes):
        c -= cz(xc, 9, -1, 9, d)
    c += bx(xs[-1] - 4, xs[-1] + 4, 9 + D_FLAT, 13, 0, 8)           # last hole is the spool's D-bore
    c += bx(0, 30, 18, 32, 0, 16)                                    # side block for horizontal bores
    c -= cy(9, 8, 17, 33, 11.2)
    c -= cy(23, 8, 17, 33, 3.5)
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

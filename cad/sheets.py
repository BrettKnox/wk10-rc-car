"""Engineering drawing sheets, chart data and viewer meshes for the WK-10 review page.

Run:  python cad/sheets.py
Writes review/sheets/*.svg (hidden-line-removed orthographic views, third-angle, mm),
review/data.json (steering sweep, bump steer, gearing, masses, parts) and review/models/*.stl.
Every dimension is computed from the same wk10.py parameters the parts are built from.
"""
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import wk10 as w  # noqa: E402
from build123d import Compound, export_stl  # noqa: E402

ROOT = Path(__file__).parent.parent / "review"
REV, DATE = "D", "2026-10-12"

VIEWS = {  # camera origin, up; projected (u, v) = top:(X,Y) side:(X,Z) end:(Y,Z)
    "top": ((0, 0, 2000), (0, 1, 0)),
    "side": ((0, -2000, 0), (0, 0, 1)),
    "end": ((2000, 0, 0), (0, 0, 1)),
}


def uv(view, p):
    x, y, z = p
    return {"top": (x, y), "side": (x, z), "end": (y, z)}[view]


def edge_points(e):
    try:
        straight = e.geom_type.name == "LINE"
    except Exception:
        straight = False
    n = 1 if straight else max(8, int(e.length / 1.2))
    return [tuple(e @ (i / n))[:2] for i in range(n + 1)]


class Sheet:
    def __init__(self, num, title, w_, h_):
        self.num, self.title, self.W, self.H = num, title, w_, h_
        self.items, self.views = [], {}

    # ---- views
    def view(self, key, kind, shape, s, ox, oy, u0, v0, hidden=False, label=None):
        """Place `shape` projected in `kind` with scale s; (u0, v0) maps to sheet (ox, oy)."""
        self.views[key] = (kind, s, ox, oy, u0, v0)
        vis, hid = shape.project_to_viewport(*VIEWS[kind], (0, 0, 0))
        top, left = 1e9, 1e9
        for cls, edges in (("vis", vis), ("hid", hid if hidden else [])):
            d = []
            for e in edges:
                pts = [self.to(key, p) for p in edge_points(e)]
                top = min(top, min(y for _, y in pts))
                left = min(left, min(x for x, _ in pts))
                d.append("M" + " L".join(f"{x:.2f} {y:.2f}" for x, y in pts))
            if d:
                self.items.append(f'<path class="{cls}" d="{" ".join(d)}"/>')
        if label:
            self.items.append(f'<text class="vlabel" x="{left:.1f}" y="{top - 14:.1f}">{label}</text>')

    def to(self, key, p_uv):
        kind, s, ox, oy, u0, v0 = self.views[key]
        u, v = p_uv
        return ox + s * (u - u0), oy - s * (v - v0)

    def pt(self, key, p3):
        return self.to(key, uv(self.views[key][0], p3))

    # ---- annotation
    def line(self, a, b, cls="dim"):
        self.items.append(f'<line class="{cls}" x1="{a[0]:.2f}" y1="{a[1]:.2f}" x2="{b[0]:.2f}" y2="{b[1]:.2f}"/>')

    def text(self, x, y, s, cls="dimt", anchor="middle", rot=0):
        tr = f' transform="rotate({rot} {x:.2f} {y:.2f})"' if rot else ""
        self.items.append(f'<text class="{cls}" x="{x:.2f}" y="{y:.2f}" text-anchor="{anchor}"{tr}>{s}</text>')

    def arrow(self, tip, toward):
        dx, dy = toward[0] - tip[0], toward[1] - tip[1]
        L = math.hypot(dx, dy) or 1
        ux, uy = dx / L, dy / L
        a, b = 7.0, 2.4
        p1 = (tip[0] + ux * a - uy * b, tip[1] + uy * a + ux * b)
        p2 = (tip[0] + ux * a + uy * b, tip[1] + uy * a - ux * b)
        self.items.append(f'<path class="arr" d="M{tip[0]:.2f} {tip[1]:.2f} L{p1[0]:.2f} {p1[1]:.2f} L{p2[0]:.2f} {p2[1]:.2f} Z"/>')

    def dim(self, key, p1, p2, orient, off, txt=None, raw=False):
        """Linear dimension between 3D points (or (u,v) if raw). orient 'h' / 'v' / 'a' (aligned).
        off = signed offset in sheet units, perpendicular to the measured direction."""
        a = self.to(key, p1) if raw else self.pt(key, p1)
        b = self.to(key, p2) if raw else self.pt(key, p2)
        kind, s, *_ = self.views[key]
        ua, ub = (p1, p2) if raw else (uv(kind, p1), uv(kind, p2))
        if orient == "h":
            val = abs(ub[0] - ua[0]); y = min(a[1], b[1]) - off if off > 0 else max(a[1], b[1]) - off
            da, db = (a[0], y), (b[0], y)
        elif orient == "v":
            val = abs(ub[1] - ua[1]); x = max(a[0], b[0]) + off if off > 0 else min(a[0], b[0]) + off
            da, db = (x, a[1]), (x, b[1])
        else:
            val = math.dist(ua, ub)
            dx, dy = b[0] - a[0], b[1] - a[1]
            L = math.hypot(dx, dy)
            nx, ny = -dy / L, dx / L
            da, db = (a[0] + nx * off, a[1] + ny * off), (b[0] + nx * off, b[1] + ny * off)
        self.line(a, da, "ext")
        self.line(b, db, "ext")
        self.line(da, db)
        self.arrow(da, db)
        self.arrow(db, da)
        label = txt or (f"{val:.2f}".rstrip("0").rstrip("."))
        mx, my = (da[0] + db[0]) / 2, (da[1] + db[1]) / 2
        if orient == "v":
            self.text(mx - 5, my, label, rot=-90)
        elif orient == "a":
            ang = math.degrees(math.atan2(db[1] - da[1], db[0] - da[0]))
            if ang > 90 or ang < -90:
                ang += 180
            self.text(mx, my - 5, label, rot=ang)
        else:
            self.text(mx, my - 5, label)

    def callout(self, key, p3, txt, dx, dy, raw=False):
        a = self.to(key, p3) if raw else self.pt(key, p3)
        b = (a[0] + dx, a[1] + dy)
        self.line(a, b, "lead")
        self.items.append(f'<circle class="dot" cx="{a[0]:.2f}" cy="{a[1]:.2f}" r="1.8"/>')
        c = (b[0] + (28 if dx >= 0 else -28), b[1])
        self.line(b, c, "lead")
        self.text(c[0] + (2 if dx >= 0 else -2), c[1] + 4, txt, cls="dimt", anchor="start" if dx >= 0 else "end")

    def center(self, key, p1, p2):
        self.line(self.pt(key, p1), self.pt(key, p2), "ctr")

    def ground(self, key, x0, x1):
        a, b = self.pt(key, (x0, 0, 0)), self.pt(key, (x1, 0, 0))
        if self.views[key][0] == "end":
            a, b = self.pt(key, (0, x0, 0)), self.pt(key, (0, x1, 0))
        self.line(a, b, "gnd")

    def note(self, x, y, lines, cls="note"):
        for i, s in enumerate(lines):
            self.text(x, y + i * 15, s, cls=cls, anchor="start")

    def svg(self):
        W, H = self.W, self.H
        tb_w, tb_h = 380, 92
        x0, y0 = W - tb_w - 14, H - tb_h - 14
        tb = [
            f'<rect class="frame" x="8" y="8" width="{W - 16}" height="{H - 16}"/>',
            f'<rect class="tb" x="{x0}" y="{y0}" width="{tb_w}" height="{tb_h}"/>',
            f'<line class="tbl" x1="{x0}" y1="{y0 + 34}" x2="{x0 + tb_w}" y2="{y0 + 34}"/>',
            f'<line class="tbl" x1="{x0}" y1="{y0 + 63}" x2="{x0 + tb_w}" y2="{y0 + 63}"/>',
            f'<line class="tbl" x1="{x0 + 250}" y1="{y0 + 34}" x2="{x0 + 250}" y2="{y0 + tb_h}"/>',
            f'<text class="tbt" x="{x0 + 10}" y="{y0 + 23}">{self.title}</text>',
            f'<text class="tbs" x="{x0 + 10}" y="{y0 + 53}">WK-10 1/10 PAN CAR  ·  UNITS mm  ·  3RD ANGLE</text>',
            f'<text class="tbs" x="{x0 + 10}" y="{y0 + 82}">SOURCE cad/wk10.py  ·  {DATE}</text>',
            f'<text class="tbn" x="{x0 + 260}" y="{y0 + 55}">WK10-{self.num}</text>',
            f'<text class="tbs" x="{x0 + 260}" y="{y0 + 82}">REV {REV}  ·  NTS</text>',
        ]
        body = "\n".join(self.items + tb)
        return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" class="sheet" '
                f'role="img" aria-label="Drawing WK10-{self.num}: {self.title}">\n{body}\n</svg>')


def build():
    ps, r = w.parts(), w.refs()
    solid = {k: v[0] for k, v in ps.items()}
    asm_parts = [s for k, s in solid.items() if "_x" not in k]
    refs_main = [r[k] for k in ("tires", "rims", "battery", "motor", "spur", "pinion", "servo", "servo_saver", "esc",
                                "esp32", "rear_axle", "kingpins", "collars", "coupling", "clamp_hexes",
                                "shocks", "foam_bumper", "tie_rod_cups", "tie_rods")]
    whole = Compound(asm_parts + refs_main)
    sheets = []

    # ------------------------------------------------------------ 01 general arrangement
    S = Sheet("01", "GENERAL ARRANGEMENT", 1000, 760)
    s = 1.45
    bb = whole.bounding_box()
    S.view("top", "top", whole, s, 60, 90, bb.min.X, bb.max.Y, label="TOP VIEW")
    S.view("side", "side", whole, s, 60, 470 + s * bb.max.Z, bb.min.X, 0, label="RIGHT ELEVATION")
    S.view("end", "end", whole, s, 600, 470 + s * bb.max.Z, -bb.max.Y, 0, label="FRONT VIEW (FROM +X)")
    # top
    S.dim("top", (bb.min.X, bb.max.Y, 0), (bb.max.X, bb.max.Y, 0), "h", 18, f"{bb.max.X - bb.min.X:.1f} OVERALL")
    S.dim("top", (w.RA_X, -bb.max.Y, 0), (w.FA_X, -bb.max.Y, 0), "h", -22, f"{w.WB:.0f} WHEELBASE")
    S.dim("top", (bb.max.X, -bb.max.Y, 0), (bb.max.X, bb.max.Y, 0), "v", 70, f"{2 * bb.max.Y:.1f} OVERALL")
    S.dim("top", (w.FA_X, -w.TRACK / 2, 0), (w.FA_X, w.TRACK / 2, 0), "v", s * (bb.max.X - w.FA_X) + 30, f"{w.TRACK:.0f} TRACK")
    for x in (w.FA_X, w.RA_X):
        S.center("top", (x, -bb.max.Y - 6, 0), (x, bb.max.Y + 6, 0))
    # side
    S.ground("side", bb.min.X - 10, bb.max.X + 10)
    S.dim("side", (bb.min.X, 0, 0), (bb.min.X, 0, bb.max.Z), "v", -22, f"{bb.max.Z:.0f}")
    S.dim("side", (w.FA_X, 0, 0), (w.FA_X, 0, w.AXLE_Z), "v", 60, f"{w.AXLE_Z:.1f} AXLE")
    S.dim("side", (0, 0, 0), (0, 0, w.FLOOR_Z), "v", 26, f"{w.FLOOR_Z:.0f} RIDE")
    S.callout("side", (w.RA_X + w.R_W * 0.7, 0, w.AXLE_Z + w.R_W * 0.7), f"TIRE Ø{2 * w.R_W:.0f} × {w.W_W:.0f}", 30, -40)
    S.callout("side", (w.MOTOR_X, 0, w.AXLE_Z + w.MOTOR_D / 2), "MOTOR 3650", 60, -52)
    # end
    S.ground("end", -bb.max.Y - 10, bb.max.Y + 10)
    S.dim("end", (0, -w.TRACK / 2, 0), (0, w.TRACK / 2, 0), "h", -18, f"{w.TRACK:.0f}")
    S.note(60, 690, ["BODY REMOVED (SHEET 06). BOUGHT PARTS MODELLED FROM MAKER DIMENSIONS: TIRES + RIMS, QR3650,",
                     "10BL60 G2 ESC, MG996R, ESP32-DEVKITC V4, 2S PACK, 87T/14T 48P, AXLE, KINGPINS, COLLARS"])
    sheets.append(S)

    # ------------------------------------------------------------ 02 front suspension + steering
    front_keys = ["front_end", "upper_arm_L", "upper_arm_R", "knuckle_L", "knuckle_R", "splice_servo_block",
                  "steer_bar", "bump_stops"]
    fw = [w.wheel(w.FA_X, sd * w.TRACK / 2) for sd in (1, -1)]
    front = Compound([solid[k] for k in front_keys] + [r["servo"], r["servo_saver"], r["kingpins"],
                                                        r["tie_rod_cups"], r["tie_rods"], r["foam_bumper"]] + fw)
    S = Sheet("02", "FRONT SUSPENSION + STEERING", 1000, 760)
    s = 2.0
    S.view("top", "top", front, s, 40, 60, 80, 102, label="TOP VIEW")
    S.center("top", (w.FA_X, -105, 0), (w.FA_X, 105, 0))
    S.center("top", (w.KP_X, -105, 0), (w.KP_X, 105, 0))
    S.dim("top", (w.KP_X, -w.KINGPIN_Y, 0), (w.KP_X, w.KINGPIN_Y, 0), "v", 150, f"{2 * w.KINGPIN_Y:.0f} KINGPINS")
    S.dim("top", (w.FA_X, w.KINGPIN_Y, 0), (w.KP_X, w.KINGPIN_Y, 0), "h", -150, f"{w.KP_TRAIL:.1f} TRAIL")
    hb = w.horn_ball(1)
    S.dim("top", (w.SERVO_X, 0, 0), (hb[0], 0, 0), "h", -30, f"{w.SAVER_R:.0f} SAVER")
    S.dim("top", (hb[0], -hb[1], 0), (hb[0], hb[1], 0), "v", -40, f"{2 * w.BAR_Y:.0f} BALLS")
    tip = w.arm_tip(1)
    S.dim("top", hb + (0,), tip + (0,), "a", -16, f"{math.dist(hb, tip):.1f} ROD")
    S.callout("top", (w.KP_X, -w.KINGPIN_Y, 0), "KINGPIN Ø3 HARDENED, BRASS BUSH", 30, 50)
    S.callout("top", (w.SERVO_X, 0, 0), "LOW-PROFILE SERVO + SAVER", 30, 90)
    S.callout("top", (w.BUMPER_X + 10, 40, 0), "FOAM BUMPER", 20, -40)
    corner = Compound([solid[k] for k in ("front_end", "upper_arm_R", "knuckle_R", "bump_stops")] +
                      [r["kingpins"], r["bushes"]])
    S.view("side", "side", corner, 4.0, 560, 560, 104, 0, label="RIGHT CORNER, ELEVATION")
    S.line(S.to("side", (100, 0)), S.to("side", (190, 0)), "gnd")
    kx = w.KP_X
    S.dim("side", (kx, 0, w.KP_Z0), (kx, 0, w.KP_Z1), "v", -70, f"{w.KP_Z1 - w.KP_Z0:.0f} KINGPIN")
    S.dim("side", (kx, 0, w.KNUCKLE_Z[0]), (kx, 0, w.KNUCKLE_Z[1]), "v", 50, f"{w.KNUCKLE_Z[1] - w.KNUCKLE_Z[0]:.0f} BUSH")
    S.dim("side", (kx, 0, w.KNUCKLE_Z[1]), (kx, 0, w.UPPER_Z[0]), "v", 50, f"{w.UPPER_Z[0] - w.KNUCKLE_Z[1]:.0f} SPRING")
    S.dim("side", (w.FA_X, 0, 0), (w.FA_X, 0, w.AXLE_Z), "v", -110, f"{w.AXLE_Z:.1f}")
    st = w.steering()
    st.pop("_lock_poses", None)
    L = st["left"]
    S.note(40, 640, [f"ACKERMANN {L['ackermann_pct']}% BOTH WAYS · INNER {L['inner_deg']}° / OUTER {L['outer_deg']}° (IDEAL {L['ideal_outer_deg']}°)",
                     f"SERVO {L['servo_deg']}° FOR 28° LOCK · MIN TRANSMISSION {L['min_transmission_deg']}° · SCRUB {w.TRACK / 2 - w.KINGPIN_Y:.0f}",
                     f"BUMP STEER {st['bump_steer_deg_per_3mm']}° / 3 mm, {st['bump_steer_deg_full']}° AT FULL TRAVEL",
                     "TIE RODS: HOT RACING CUPS + 5.8 BALLS ON CUT M3 STAINLESS · SAVER/SERVO HOLES: VERIFY"])
    sheets.append(S)

    # ------------------------------------------------------------ 03 rear pod + drivetrain
    pod_keys = ["pod_floor", "pod_plate_L", "pod_plate_R_motor", "pod_brace", "spur_carrier"]
    pod = Compound([solid[k] for k in pod_keys] + [r["motor"], r["spur"], r["pinion"], r["rear_axle"],
                                                   r["coupling"], r["shocks"]])
    S = Sheet("03", "REAR POD + DRIVETRAIN", 1000, 760)
    s = 2.7
    side_pod = Compound([solid[k] for k in pod_keys] + [r["motor"], r["spur"], r["pinion"], r["shocks"]])
    S.view("side", "side", side_pod, s, 60, 420, -200, 0, label="RIGHT ELEVATION (MOTOR SIDE, COVER OFF)")
    S.line(S.to("side", (-195, 0)), S.to("side", (-75, 0)), "gnd")
    S.center("side", (w.RA_X, 0, 0), (w.RA_X, 0, 70))
    S.dim("side", (w.MOTOR_X, 0, w.AXLE_Z), (w.RA_X, 0, w.AXLE_Z), "h", 58, f"{w.CENTRE:.2f} C-C")
    S.dim("side", (w.RA_X, 0, w.PIVOT_Z), (w.PIVOT_X, 0, w.PIVOT_Z), "h", -46, f"{w.PIVOT_X - w.RA_X:.1f} PIVOT")
    S.dim("side", (w.RA_X, 0, 0), (w.RA_X, 0, w.AXLE_Z), "v", -90, f"{w.AXLE_Z:.1f}")
    S.dim("side", (w.SHOCK_X, 0, w.SHOCK_Z0), (w.SHOCK_X, 0, w.SHOCK_Z0 + w.SHOCK_L), "v", 40, f"{w.SHOCK_L:.0f} SHOCK")
    S.callout("side", (w.RA_X, 0, w.AXLE_Z + w.SPUR_D / 2), f"SPUR 87T 48P Ø{w.SPUR_D:.1f}", 10, -38)
    S.callout("side", (w.MOTOR_X, 0, w.AXLE_Z), f"PINION {w.PINION_T}T · {w.SPUR_T / w.PINION_T:.2f}:1", 20, 110)
    S.callout("side", (w.PIVOT_X, 0, w.PIVOT_Z), "PIVOT M4 SHOULDER Ø5", 20, 60)
    S.view("top", "top", pod, 2.3, 560, 70, -200, 110, label="TOP VIEW")
    yy = w.POD_PLATE_Y
    S.dim("top", (w.RA_X + 16, -yy, 0), (w.RA_X + 16, yy, 0), "v", 70, f"{2 * yy:.0f} IN-PLATES")
    S.dim("top", (w.SHOCK_X, -w.SHOCK_Y, 0), (w.SHOCK_X, w.SHOCK_Y, 0), "v", -90, f"{2 * w.SHOCK_Y:.0f} SHOCKS")
    S.note(40, 640, [f"GEAR MESH {w.PINION_T}T/{w.SPUR_T}T 48P (m 0.529): CENTRE {w.CENTRE:.3f}, SLOTS ±{w.MESH_TRAVEL} FIT 13T-17T",
                     "SPUR ON A 5 mm ALUMINIUM FLANGE COUPLING (2 SET SCREWS ON A FLAT) + PRINTED CARRIER, 4 × M3",
                     "TWO INJORA 53 mm COILOVERS, SYMMETRIC; POD SWINGS ±4° (CHECKED INSIDE SHOCK TRAVEL)",
                     "685 SEATS PRESSED FROM OUTBOARD AGAINST A 1 mm INBOARD LIP; GEAR COVER OPEN UNDERNEATH"])
    sheets.append(S)

    # ------------------------------------------------------------ 04 knuckle detail
    kn = solid["knuckle_L"]
    S = Sheet("04", "KNUCKLE (LEFT) DETAIL", 1000, 820)
    s = 7.0
    S.view("top", "top", kn, s, 80, 70, 104, 82, hidden=True, label="TOP VIEW")
    S.view("side", "side", kn, s, 80, 700, 104, 14, hidden=True, label="ELEVATION (BORE AXIS INTO PAGE)")
    S.callout("top", (w.KP_X, w.KINGPIN_Y + 2, 0), f"Ø{w.BUSH_OD + w.HOLE_COMP:.1f} BRASS-BUSH PRESS", 40, -30)
    S.dim("top", (w.FA_X, w.KINGPIN_Y, 0), (w.FA_X, w.KINGPIN_Y + w.H_LEN, 0), "v", -60, f"{w.H_LEN:.0f} HOUSING")
    S.dim("top", (w.FA_X, w.KINGPIN_Y - 8, 0), (w.KP_X, w.KINGPIN_Y - 8, 0), "h", -20, f"{w.KP_TRAIL:.1f} TRAIL")
    S.dim("top", (tip[0], tip[1], 0), (w.KP_X, tip[1], 0), "h", 34, f"{w.STEER_ARM:.0f} ARM")
    S.callout("side", (w.FA_X + w.BRG_OD / 2 * 0.7, 0, w.AXLE_Z + w.BRG_OD / 2 * 0.7),
              f"Ø{w.BRG_OD:.1f} THRU FROM OUTBOARD: 685 + 3 mm SPACER + 685", 50, -40)
    S.callout("side", (w.FA_X, 0, w.AXLE_Z - 3.5), "Ø9 LIP × 1 INBOARD", 60, 30)
    S.dim("side", (w.KP_X, 0, w.KNUCKLE_Z[0]), (w.KP_X, 0, w.KNUCKLE_Z[1]), "v", 60, f"{w.KNUCKLE_Z[1] - w.KNUCKLE_Z[0]:.0f}")
    S.note(560, 100, ["MATERIAL PETG, 100% INFILL", "PRINT BARREL UPRIGHT, ARM IN THE LAYERS",
                      "BALL STUD: M3 INSERT IN THE ARM TOP", f"BORES MODELLED +{w.HOLE_COMP} (CALIBRATE ON COUPON)",
                      "STUB: HARDENED Ø5, COLLAR INBOARD,", "LIP SHIM 1.3 + RACE SHIM 1.0", "UNTOLERANCED: ±0.2"])
    sheets.append(S)

    # ------------------------------------------------------------ 05 spur carrier + gear cover
    S = Sheet("05", "SPUR CARRIER, COUPLING, GEAR COVER", 1000, 760)
    sp = Compound([solid["spur_carrier"], r["coupling"]])
    s = 6.0
    S.view("end", "side", sp, s, 80, 330, w.RA_X - 15, w.AXLE_Z - 15, hidden=True, label="CARRIER + COUPLING, AXIAL")
    S.callout("end", (w.RA_X + 8, 0, w.AXLE_Z), "4× Ø3.4 ON Ø16 BC (COUPLING PATTERN)", 70, -60)
    S.view("top", "top", sp, s, 80, 420, w.RA_X - 15, w.SPUR_FACE + 22, hidden=True, label="CARRIER + COUPLING, PROFILE")
    S.dim("top", (w.RA_X + 16, w.SPUR_FACE, 0), (w.RA_X + 16, w.SPUR_FACE + w.SPUR_W, 0), "v", 30, f"{w.SPUR_W:.0f} SPUR")
    gc = solid["gear_cover"]
    S.view("gc", "side", gc, 3.0, 600, 330, w.MOTOR_X - 12, 8, label="GEAR COVER, OUTBOARD VIEW")
    S.note(600, 470, ["SPUR: DRILL CENTRE Ø10.2 ON THE CARRIER SPIGOT,", "4× Ø3.2 USING THE CARRIER AS A TEMPLATE",
                      "M3×16 THROUGH COUPLING, SPUR, CARRIER; NUTS IN POCKETS", "AXLE: HARDENED Ø5, FLAT UNDER THE SET SCREWS",
                      "COVER: 2 × M3 INTO THE MOTOR PLATE, OPEN BELOW"])
    sheets.append(S)

    # ------------------------------------------------------------ 06 body + envelope
    S = Sheet("06", "BODY, POSTS + ENVELOPE", 1000, 760)
    shell, windows, wing = r["body"], r["windows"], r["wing"]
    env = Compound([shell, windows, wing, r["tires"], solid["front_end"], solid["pan_rear"], r["body_posts"]])
    bb = env.bounding_box()
    s = 1.15
    S.view("side", "side", env, s, 60, 90 + s * bb.max.Z, bb.min.X, 0, label="RIGHT ELEVATION, BODY ON")
    S.view("end", "end", env, s, 680, 90 + s * bb.max.Z, -bb.max.Y, 0, label="FRONT VIEW")
    S.view("top", "top", Compound([shell, windows, wing, r["tires"]]), s, 60, 330, bb.min.X, bb.max.Y, label="TOP VIEW")
    S.ground("side", bb.min.X - 10, bb.max.X + 10)
    S.dim("side", (bb.min.X, 0, bb.max.Z), (bb.max.X, 0, bb.max.Z), "h", -16, f"{bb.max.X - bb.min.X:.0f} OVERALL")
    S.dim("side", (bb.max.X, 0, 0), (bb.max.X, 0, bb.max.Z), "v", 24, f"{bb.max.Z:.0f}")
    S.dim("side", (w.RA_X, 0, 0), (w.FA_X, 0, 0), "h", 22, f"{w.WB:.0f} WB")
    S.ground("end", -bb.max.Y - 10, bb.max.Y + 10)
    S.dim("end", (0, -w.bought.BODY["W"] / 2, 0), (0, w.bought.BODY["W"] / 2, 0), "h", -18, f"{w.bought.BODY['W']:.0f} BODY")
    pf = w.body_post_top(w.BODY_POST_X_F, w.BODY_POST_Y_F)
    pr = w.body_post_top(w.BODY_POST_X_R, w.BODY_POST_Y_R)
    S.note(620, 380, ["BODY: PROTOFORM VENTURI GT, 190 mm PC",
                      "430 x 191, 258 WB, 85-90 g PAINTED",
                      "HOOD/DECK HEIGHTS: STAND-IN, NOT PUBLISHED.",
                      "OFFER THE SHELL UP BEFORE TRIMMING.",
                      f"FRONT POST TOPS Z {pf:.1f}, REAR Z {pr:.1f}",
                      "(SKIN + 9). R-CLIP HOLE AT SKIN + 4.",
                      "POSTS: 6 x 58 M3 STANDOFFS ON PRINTED RISERS,",
                      "ALL FOUR ON THE PAN (NOT THE POD).",
                      "ARCHES: TIRE + 9, STRETCHED 12 FORE/AFT,",
                      "OPEN TO THE SKIRT. CLEAR LOCK + BUMP."])
    sheets.append(S)
    return ps, r, solid, sheets


def data(ps, r, solid):
    st = w.steering()
    st.pop("_lock_poses", None)
    # full steering sweep (same branch-tracked solver as the check)
    L = {sd: math.dist(w.horn_ball(sd), w.arm_tip(sd)) for sd in (1, -1)}

    def track(sd, t, dz=0.0, steps=120):
        p = 0.0
        for i in range(1, steps + 1):
            h = w.horn_ball(sd, t * i / steps)
            f = lambda q: math.dist(h, w.arm_tip(sd, q)) ** 2 + dz * dz - L[sd] ** 2
            for _ in range(40):
                d = (f(p + 1e-7) - f(p - 1e-7)) / 2e-7
                p -= f(p) / d
        return p
    sweep = []
    for deg in range(0, 37, 2):
        t = math.radians(deg)
        pl, pr = math.degrees(track(1, t)), math.degrees(track(-1, t))
        inner, outer = max(abs(pl), abs(pr)), min(abs(pl), abs(pr))
        ideal = math.degrees(math.atan(1 / (1 / math.tan(math.radians(inner)) + 2 * w.KINGPIN_Y / w.WB))) if inner > 0.01 else 0
        sweep.append({"servo": deg, "inner": round(inner, 2), "outer": round(outer, 2), "ideal": round(ideal, 2)})
    bump = [{"dz": dz, "toe": round(math.degrees(track(1, 0.0, dz, steps=1)), 3)} for dz in range(-5, 8)]
    dens = {"PLA": 1.24, "PETG": 1.27}
    parts = []
    for k, (shape, note) in ps.items():
        mat = "PETG" if note.startswith("PETG") else "PLA"
        fill = 1.0 if "100%" in note else 0.6
        qty = int(k.rsplit("_x", 1)[1]) if "_x" in k else 1
        bb = shape.bounding_box().size
        parts.append({"name": k, "note": note, "material": mat, "qty": qty,
                      "volume_cm3": round(shape.volume / 1000, 2),
                      "mass_g": round(shape.volume / 1000 * dens[mat] * fill, 1),
                      "bbox": [round(bb.X, 1), round(bb.Y, 1), round(bb.Z, 1)]})
    gearing = [{"pinion": 13, "ratio": 6.69, "top_kmh": 49.0}, {"pinion": 14, "ratio": 6.21, "top_kmh": 52.6},
               {"pinion": 15, "ratio": 5.80, "top_kmh": 56.0}, {"pinion": 16, "ratio": 5.44, "top_kmh": 59.2},
               {"pinion": 17, "ratio": 5.12, "top_kmh": 62.6}]
    # bought masses from the makers' pages (bought.py); printed from the CAD volumes above
    printed = round(sum(p["mass_g"] * p["qty"] for p in parts))
    masses = [("2S LiPo", 256), ("Wheels, hubs, axles", 225), ("Printed parts", printed), ("Motor", 186), ("Body + wing", 90),
              ("ESC", 67), ("Servo", 55), ("ESP32 + leads", 45), ("Hardware", 40),
              # ponytail: estimates, makers publish no masses; weigh on arrival and replace
              ("Shocks + coupling (est.)", 35), ("Saver, cups, rods, bumper, posts (est.)", 40)]
    # CG: every mass at its own centroid (printed parts from CAD, bought parts at their modelled shape)
    from build123d import CenterOf
    pts = []
    for p in parts:
        if "_x" in p["name"]:
            pts.append((p["mass_g"] * p["qty"], (0, 0, w.AXLE_Z)))           # hubs, shims: at the axles, near enough
        else:
            c = solid[p["name"]].center(CenterOf.MASS)
            pts.append((p["mass_g"], (c.X, c.Y, c.Z)))
    bought_at = {"2S LiPo": "battery", "Wheels, hubs, axles": "tires", "Motor": "motor", "Body + wing": "body",
                 "ESC": "esc", "Servo": "servo", "ESP32 + leads": "esp32", "Hardware": "battery",
                 "Shocks + coupling (est.)": "shocks", "Saver, cups, rods, bumper, posts (est.)": "servo_saver"}
    for item, g in masses:
        if item in bought_at:
            c = r[bought_at[item]].center(CenterOf.BOUNDING_BOX)
            pts.append((g, (c.X, c.Y, c.Z)))
    m = sum(g for g, _ in pts)
    cg = [sum(g * q[i] for g, q in pts) / m for i in range(3)]
    rear = (w.FA_X - cg[0]) / w.WB
    return {"steering": st, "sweep": sweep, "bump": bump, "parts": parts, "gearing": gearing,
            "masses": [{"item": a, "g": b} for a, b in masses],
            "cg": {"mass_g": round(m), "x": round(cg[0], 1), "y": round(cg[1], 1), "z": round(cg[2], 1),
                   "rear_pct": round(rear * 100), "tip_g": round(w.TRACK / 2 / cg[2], 2)}}


GROUPS = {  # viewer colour groups
    "chassis": ["pan_rear", "pan_front", "splice_servo_block", "esp32_cradle", "esp32_lid", "battery_stop"],
    "front": ["front_end", "upper_arm_L", "upper_arm_R", "knuckle_L", "knuckle_R", "bump_stops"],
    "steering": ["steer_bar"],
    "rear": ["pod_floor", "pod_plate_L", "pod_plate_R_motor", "pod_brace", "spur_carrier", "gear_cover"],
}
REF_MODELS = {  # bought parts: viewer group, colour, opacity
    "tires": ("wheels", "#2b2d30", 1), "rims": ("wheels", "#c9ced3", 1),
    "battery": ("electronics", "#1d1f22", 1), "esc": ("electronics", "#2a2d31", 1),
    "esp32": ("electronics", "#14181c", 1), "esp32_pins": ("electronics", "#c8a24a", 1),
    "servo": ("electronics", "#1f2124", 1), "servo_saver": ("steering", "#2b5fb8", 1),
    "tie_rod_cups": ("steering", "#1c1c1c", 1), "tie_rods": ("steering", "#b8bcc2", 1), "ball_studs": ("steering", "#c9ced3", 1),
    "leads": ("electronics", "#b8432f", 1),
    "motor": ("drivetrain", "#4a5058", 1), "spur": ("drivetrain", "#ebe7dc", 1), "pinion": ("drivetrain", "#b8bcc2", 1),
    "rear_axle": ("drivetrain", "#9aa3ab", 1), "collars": ("drivetrain", "#9aa3ab", 1), "kingpins": ("front", "#9aa3ab", 1),
    "coupling": ("drivetrain", "#c4c9ce", 1), "clamp_hexes": ("wheels", "#2b5fb8", 1), "front_axles": ("front", "#9aa3ab", 1),
    "bushes": ("front", "#c8a24a", 1), "shocks": ("rear", "#d4a017", 1), "foam_bumper": ("chassis", "#5d6166", 1),
    "body_posts": ("body", "#9aa3ab", 1),
    "body": ("body", "#dfe6ec", 0.32), "windows": ("body", "#1c2329", 0.5), "wing": ("body", "#dfe6ec", 0.45),
}


if __name__ == "__main__":
    (ROOT / "sheets").mkdir(parents=True, exist_ok=True)
    (ROOT / "models").mkdir(parents=True, exist_ok=True)
    ps, r, solid, sheets = build()
    for S in sheets:
        (ROOT / "sheets" / f"WK10-{S.num}.svg").write_text(S.svg(), encoding="utf-8")
    manifest = {}
    for g, keys in GROUPS.items():
        for k in keys:
            export_stl(solid[k], str(ROOT / "models" / f"{k}.stl"), tolerance=0.04, angular_tolerance=0.25)
            manifest[k] = {"g": g}
    r["leads"] = w.leads()
    for k, (g, c, o) in REF_MODELS.items():
        export_stl(r[k], str(ROOT / "models" / f"ref_{k}.stl"), tolerance=0.05, angular_tolerance=0.3)
        manifest["ref_" + k] = {"g": g, "c": c, "o": o}
    d = data(ps, r, solid)
    d["models"] = manifest
    d["body_h"] = round(Compound([r["body"], r["wing"]]).bounding_box().max.Z)
    (ROOT / "data.json").write_text(json.dumps(d, indent=1), encoding="utf-8")
    print("sheets", [S.num for S in sheets], "models", len(manifest))

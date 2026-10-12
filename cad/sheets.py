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
    refs_main = [r[k] for k in ("tires", "rims", "battery", "motor", "spur", "pinion", "servo", "servo_horn", "esc",
                                "esp32", "rear_axle", "kingpins", "collars")]
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
                  "tie_rod_L_horn", "tie_rod_L_knuckle", "tie_rod_R_horn", "tie_rod_R_knuckle", "bump_stops"]
    fw = [w.wheel(w.FA_X, sd * w.TRACK / 2) for sd in (1, -1)]
    front = Compound([solid[k] for k in front_keys] + [r["servo"], r["kingpins"]] + fw)
    S = Sheet("02", "FRONT SUSPENSION + STEERING", 1000, 760)
    s = 2.3
    S.view("top", "top", front, s, 40, 60, 64, 102, label="TOP VIEW")
    S.center("top", (w.FA_X, -105, 0), (w.FA_X, 105, 0))
    S.dim("top", (w.FA_X, -w.KINGPIN_Y, 0), (w.FA_X, w.KINGPIN_Y, 0), "v", 120, f"{2 * w.KINGPIN_Y:.0f} KINGPINS")
    S.dim("top", (w.SERVO_X, 0, 0), (w.SERVO_X + w.HORN_R, 0, 0), "h", -26, f"{w.HORN_R:.0f} HORN")
    tip = w.arm_tip(1)
    S.dim("top", (tip[0], tip[1], 0), (w.FA_X, tip[1], 0), "h", 22, f"{w.STEER_ARM:.0f}")
    S.dim("top", (w.SERVO_X + w.HORN_R, 0, 0), (tip[0], tip[1], 0), "a", -16, f"{math.dist((w.SERVO_X + w.HORN_R, 0), tip):.1f} ROD")
    S.callout("top", (w.FA_X, -w.KINGPIN_Y, 0), "KINGPIN Ø3 SS", 30, 40)
    S.callout("top", (w.SERVO_X, 0, 0), "SERVO SPLINE", 30, 70)
    # right corner elevation
    corner = Compound([solid[k] for k in ("front_end", "upper_arm_R", "knuckle_R", "bump_stops")] + [r["kingpins"]])
    S.view("side", "side", corner, 4.6, 560, 520, 104, 0, label="RIGHT CORNER, ELEVATION")
    S.line(S.to("side", (100, 0)), S.to("side", (162, 0)), "gnd")
    kx = w.FA_X
    S.dim("side", (kx, 0, w.KP_Z0), (kx, 0, w.KP_Z1), "v", -78, f"{w.KP_Z1 - w.KP_Z0:.0f} KINGPIN")
    S.dim("side", (kx, 0, w.KNUCKLE_Z[0]), (kx, 0, w.KNUCKLE_Z[1]), "v", 60, f"{w.KNUCKLE_Z[1] - w.KNUCKLE_Z[0]:.0f}")
    S.dim("side", (kx, 0, w.KNUCKLE_Z[1]), (kx, 0, w.UPPER_Z[0]), "v", 60, f"{w.UPPER_Z[0] - w.KNUCKLE_Z[1]:.0f} SPRING")
    S.dim("side", (kx, 0, 0), (kx, 0, w.AXLE_Z), "v", 112, f"{w.AXLE_Z:.1f}")
    S.dim("side", (kx, 0, 0), (kx, 0, w.LOWER_Z[1]), "v", -40, f"{w.LOWER_Z[1]:.0f}")
    st = w.steering()
    st.pop("_lock_poses", None)
    L = st["left"]
    S.note(40, 620, [f"ACKERMANN {L['ackermann_pct']}% BOTH WAYS · INNER {L['inner_deg']}° / OUTER {L['outer_deg']}° (IDEAL {L['ideal_outer_deg']}°)",
                     f"SERVO {L['servo_deg']}° FOR 28° LOCK · MIN TRANSMISSION ANGLE {L['min_transmission_deg']}°",
                     f"BUMP STEER {st['bump_steer_deg_per_3mm']}° / 3 mm, {st['bump_steer_deg_full']}° AT +7 mm (TOE-OUT)",
                     f"SPRINGS: KINGPIN ID 5.5-6.5, ~16 FREE · BUMP STOP LIMITS TRAVEL TO +{w.BUMP_MM:.0f} / -{w.DROOP_MM:.0f}"])
    sheets.append(S)

    # ------------------------------------------------------------ 03 rear pod + drivetrain
    pod_keys = ["pod_floor", "pod_plate_L", "pod_plate_R_motor", "pod_brace", "spool", "spur_clamp"]
    rw = [w.wheel(w.RA_X, sd * w.TRACK / 2) for sd in (1, -1)]
    pod = Compound([solid[k] for k in pod_keys] + [r["motor"], r["spur"], r["pinion"], r["rear_axle"], r["collars"]])
    S = Sheet("03", "REAR POD + DRIVETRAIN", 1000, 760)
    s = 2.7
    side_pod = Compound([solid[k] for k in pod_keys] + [r["motor"], r["spur"], r["pinion"]])
    S.view("side", "side", side_pod, s, 60, 420, -200, 0, label="RIGHT ELEVATION (MOTOR SIDE)")
    S.line(S.to("side", (-195, 0)), S.to("side", (-75, 0)), "gnd")
    S.center("side", (w.RA_X, 0, 0), (w.RA_X, 0, 70))
    S.dim("side", (w.MOTOR_X, 0, w.AXLE_Z), (w.RA_X, 0, w.AXLE_Z), "h", 58, f"{w.CENTRE:.2f} C-C")
    S.dim("side", (w.RA_X, 0, w.PIVOT_Z), (w.PIVOT_X, 0, w.PIVOT_Z), "h", -46, f"{w.PIVOT_X - w.RA_X:.1f} PIVOT")
    S.dim("side", (w.RA_X, 0, 0), (w.RA_X, 0, w.AXLE_Z), "v", -90, f"{w.AXLE_Z:.1f}")
    S.dim("side", (w.PIVOT_X, 0, 0), (w.PIVOT_X, 0, w.PIVOT_Z), "v", 24, f"{w.PIVOT_Z:.0f}")
    S.callout("side", (w.RA_X, 0, w.AXLE_Z + w.SPUR_D / 2), f"SPUR 87T 48P Ø{w.SPUR_D:.1f}", 10, -38)
    S.callout("side", (w.MOTOR_X, 0, w.AXLE_Z), f"PINION {w.PINION_T}T · {w.SPUR_T / w.PINION_T:.2f}:1", 20, 110)
    S.callout("side", (w.PIVOT_X, 0, w.PIVOT_Z), "PIVOT M4 SHOULDER Ø5", 20, 60)
    S.view("top", "top", pod, 2.3, 560, 70, -200, 110, label="TOP VIEW")
    yy = w.POD_PLATE_Y
    S.dim("top", (w.RA_X + 16, -yy, 0), (w.RA_X + 16, yy, 0), "v", 70, f"{2 * yy:.0f} IN-PLATES")
    S.dim("top", (w.RA_X, -w.TRACK / 2 - 18, 0), (w.RA_X, w.TRACK / 2 + 18, 0), "v", -110, f"{w.TRACK + 36:.0f} AXLE")
    S.note(40, 640, [f"GEAR MESH {w.PINION_T}T/{w.SPUR_T}T 48P (m 0.529): CENTRE {w.CENTRE:.3f} mm, SLOTS ±1.5 mm FIT 13T-17T",
                     "SPOOL D-BORE KEYS ON A 0.5 mm FLAT GROUND ALONG THE AXLE; COLLARS + RACE SHIMS LOCATE IT AXIALLY",
                     "POD SWINGS ±4° ABOUT THE PIVOT (CHECKED); SPRING AT X -102 CARRIES ~9 N AT RIDE HEIGHT"])
    sheets.append(S)

    # ------------------------------------------------------------ 04 knuckle detail
    import build123d as bd
    kn = solid["knuckle_L"]
    S = Sheet("04", "KNUCKLE (LEFT) DETAIL", 1000, 820)
    s = 8.5
    S.view("top", "top", kn, s, 80, 70, 108, 74, hidden=True, label="TOP VIEW")
    S.view("side", "side", kn, s, 80, 700, 108, 18, hidden=True, label="ELEVATION (BORE AXIS INTO PAGE)")
    S.callout("top", (w.FA_X, w.KINGPIN_Y + 1.75, 0), "Ø3.5 KINGPIN BORE (SLIDE)", 40, -30)
    S.dim("top", (w.FA_X, w.KINGPIN_Y, 0), (w.FA_X, 73, 0), "v", 70, f"{73 - w.KINGPIN_Y:.0f}")
    S.dim("top", (tip[0], tip[1], 0), (w.FA_X, tip[1], 0), "h", 34, f"{w.STEER_ARM:.0f} ARM")
    S.dim("top", (tip[0], w.KINGPIN_Y, 0), (tip[0], tip[1], 0), "v", -30, f"{tip[1] - w.KINGPIN_Y:.1f}")
    S.callout("side", (w.FA_X + w.BRG_OD / 2 * 0.7, 0, w.AXLE_Z + w.BRG_OD / 2 * 0.7), f"Ø{w.BRG_OD:.1f} × 5 DP, 685ZZ PRESS, BOTH ENDS", 50, -40)
    S.callout("side", (w.FA_X, 0, w.AXLE_Z - 3.5), "Ø7 THRU", 60, 30)
    S.dim("side", (w.FA_X, 0, w.KNUCKLE_Z[0]), (w.FA_X, 0, w.KNUCKLE_Z[1]), "v", -70, f"{w.KNUCKLE_Z[1] - w.KNUCKLE_Z[0]:.0f}")
    S.dim("side", (w.FA_X - 8, 0, w.AXLE_Z), (w.FA_X - 8, 0, w.KNUCKLE_Z[0]), "v", -40, f"{w.AXLE_Z - w.KNUCKLE_Z[0]:.1f}", )
    S.note(560, 100, ["MATERIAL PETG, 100% INFILL", "PRINT AXLE BORE VERTICAL, HOUSING END DOWN",
                      "TREE SUPPORT UNDER ARM + WEB ONLY", f"BORES MODELLED +{w.HOLE_COMP} (ASSUMED SHRINK)",
                      "M3 HEAT-SET INSERT IN POST TOP (Ø4.2 × 6.3)", "UNTOLERANCED: ±0.2"])
    sheets.append(S)

    # ------------------------------------------------------------ 05 spool + hub details
    S = Sheet("05", "SPOOL, SPUR CLAMP, HEX HUB", 1000, 760)
    sp = Compound([solid["spool"], solid["spur_clamp"]])
    s = 6.0
    S.view("end", "side", sp, s, 80, 330, w.RA_X - 17, w.AXLE_Z - 17, hidden=True, label="SPOOL + CLAMP, AXIAL VIEW")
    S.callout("end", (w.RA_X + 2.6, 0, w.AXLE_Z + w.D_FLAT), f"D-BORE Ø{w.D_BORE:.2f}, FLAT {w.D_FLAT:.2f} FROM C", 70, -60)
    S.callout("end", (w.RA_X + 10, 0, w.AXLE_Z), "3× Ø3.4 ON Ø20 BC", 80, 20)
    S.dim("end", (w.RA_X - 15, 0, w.AXLE_Z - 17), (w.RA_X + 15, 0, w.AXLE_Z - 17), "h", -14, "Ø30 FLANGE")
    S.view("top", "top", sp, s, 80, 400, w.RA_X - 17, w.SPUR_FACE + 13, hidden=True, label="SPOOL + CLAMP, PROFILE")
    y_tip, y_out = -w.POD_PLATE_Y - w.POD_T, w.SPUR_FACE - 3
    S.dim("top", (w.RA_X + 18, y_out, 0), (w.RA_X + 18, y_tip, 0), "v", 30, f"{y_tip - y_out:.1f}")
    S.dim("top", (w.RA_X + 18, w.SPUR_FACE, 0), (w.RA_X + 18, w.SPUR_FACE + w.SPUR_W, 0), "v", 70, f"{w.SPUR_W:.0f} SPUR")
    hub = w.hex_hub()
    S.view("hub", "side", hub, 9.0, 600, 330, -9, -1, hidden=True, label="HEX HUB (×4), ELEVATION")
    S.dim("hub", (-8, 0, 0), (-8, 0, w.HUB_COLLAR + 5), "v", -20, f"{w.HUB_COLLAR + 5:.1f}")
    S.dim("hub", (-6, 0, 12), (6, 0, 12), "h", 18, "12 A/F")
    S.callout("hub", (0, 0, 3.5), "M3 SET BOLT + CAPTIVE NUT", 70, 30)
    S.note(600, 470, ["SPOOL, CLAMP, HUB: PETG 100%", "SPUR: DRILL CENTRE Ø12.2, 3× Ø3.2 USING THE CLAMP",
                      "AS A TEMPLATE; M3×12 + NUTS", "AXLE: 5 mm 304 SS, 0.5 mm FLAT ALONG THE SPOOL",
                      "RACE SHIM Ø6.5/Ø5.5 × 1 BETWEEN COLLAR AND BEARING"])
    sheets.append(S)

    # ------------------------------------------------------------ 06 body + envelope
    S = Sheet("06", "BODY, POSTS + ENVELOPE", 1000, 760)
    shell, windows, wing = r["body"], r["windows"], r["wing"]
    env = Compound([shell, windows, wing, r["tires"], solid["front_end"], solid["pod_brace"]])
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
    pr = w.body_post_top(w.POD_BRACE_X + 5, w.BODY_POST_Y_R)
    S.note(620, 380, ["BODY: PROTOFORM VENTURI GT, 190 mm PC",
                      "430 x 191, 258 WB, 85-90 g PAINTED",
                      "HOOD/DECK HEIGHTS: STAND-IN, NOT PUBLISHED.",
                      "OFFER THE SHELL UP BEFORE TRIMMING.",
                      f"FRONT POST TOPS Z {pf:.1f}, REAR Z {pr:.1f}",
                      "(SKIN + 9). R-CLIP HOLE AT SKIN + 4.",
                      "POST HOLES 7 mm. REAR HOLES ARE FORE-AFT",
                      "SLOTS: THE REAR POSTS RIDE THE POD.",
                      "ARCHES: TIRE + 9, STRETCHED 12 FORE/AFT,",
                      "OPEN TO THE SKIRT. CLEAR LOCK + BUMP."])
    sheets.append(S)
    return ps, r, solid, sheets


def data(ps, r, solid):
    st = w.steering()
    st.pop("_lock_poses", None)
    # full steering sweep (same branch-tracked solver as the check)
    def horn(t):
        vx, vy = w._rot(w.HORN_R, 0.0, t)
        return w.SERVO_X + vx, vy
    L = {sd: math.dist(horn(0), w.arm_tip(sd)) for sd in (1, -1)}

    def track(sd, t, dz=0.0, steps=120):
        p = 0.0
        for i in range(1, steps + 1):
            h = horn(t * i / steps)
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
        qty = 4 if k.endswith("_x4") else 3 if k.endswith("_x3") else 1
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
              ("ESC", 67), ("Servo", 55), ("ESP32 + leads", 45), ("Hardware", 40)]
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
                 "ESC": "esc", "Servo": "servo", "ESP32 + leads": "esp32", "Hardware": "battery"}
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
    "chassis": ["pan_rear", "pan_front", "splice_servo_block", "esp32_cradle", "esp32_lid"],
    "front": ["front_end", "upper_arm_L", "upper_arm_R", "knuckle_L", "knuckle_R", "bump_stops"],
    "steering": ["tie_rod_L_horn", "tie_rod_L_knuckle", "tie_rod_R_horn", "tie_rod_R_knuckle"],
    "rear": ["pod_floor", "pod_plate_L", "pod_plate_R_motor", "pod_brace", "spool", "spur_clamp"],
}
REF_MODELS = {  # bought parts: viewer group, colour, opacity
    "tires": ("wheels", "#2b2d30", 1), "rims": ("wheels", "#c9ced3", 1),
    "battery": ("electronics", "#1d1f22", 1), "esc": ("electronics", "#2a2d31", 1),
    "esp32": ("electronics", "#14181c", 1), "esp32_pins": ("electronics", "#c8a24a", 1),
    "servo": ("electronics", "#1f2124", 1), "servo_horn": ("electronics", "#e4e4e4", 1),
    "leads": ("electronics", "#b8432f", 1),
    "motor": ("drivetrain", "#4a5058", 1), "spur": ("drivetrain", "#ebe7dc", 1), "pinion": ("drivetrain", "#b8bcc2", 1),
    "rear_axle": ("drivetrain", "#9aa3ab", 1), "collars": ("drivetrain", "#9aa3ab", 1), "kingpins": ("front", "#9aa3ab", 1),
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

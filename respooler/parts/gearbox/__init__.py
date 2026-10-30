"""WS-A gearbox: 130 motor in an embracing clamp, 4-stage compound spur train (300:1).
The s4 GEAR (60T) is the final gear on the take-up shaft and is NOT built here.

Layout (global frame, see params / interfaces.md):
  Y -74..-66  rear plate P1: S1 rear 623, S2 rear 625, motor shaft clearance } one printed
  Y -66..-45  motor cradle pedestal, S3 rear post (625), spacer posts        } "gearbox_frame"
  Y -65..-61  s1: motor pinion 12T m0.8 / 48T on S1
  Y -60..-45  motor cap (2x M3 into cradle); Y -60..-57 is the boss-register wall
  Y -57..-52  s2: 12T m1.0 on S1 / 45T on S2
  Y -49..-43  s3: 12T m1.25 on S2 / 48T on S3
  Y -42..-35  front plate P2: S1 front 623, S2 front 625, keyed double-D collar round the can
  Y -41..-32  s4 pinion 12T m1.5 on S3 (meshes with the take-up 60T)
  Y -35..-24.2 S3 front bracket (625) on P2, outside FINAL_GEAR_ENVELOPE
  Y -35..-26.8 motor rear stop on P2 (bar behind the rear endcap)

Gears are keyed to D-cut steel shafts (D-bore, 0.5 mm flat) and held axially between the
bearing inner races by washers. The motor pinion is a press fit on the 2.0 mm shaft via 3
crush ribs (effective bore ~1.86 mm).
"""
from math import atan2, cos, sin, degrees, pi

import cadquery as cq

import params as P
from lib.part import Part, loc, axis_y, export
from lib.gears import gear_profile
from lib.common import FASTENERS, d_shaft_bore

# ---------------------------------------------------------------- positions
MX, MZ = P.MOTOR_AXIS
S1X, S1Z = P.SHAFT_S1
S2X, S2Z = P.SHAFT_S2
S3X, S3Z = P.SHAFT_S3
TX, TZ = P.TAKEUP_AXIS
ST1, ST2, ST3, ST4 = P.STAGES
MOTOR_Y0 = P.MOTOR_FRONT_FACE_Y + P.MOTOR["body_z"][1]   # model z=0 -> this Y (model z -> -Y)

# Y stations
P1_Y = (-74.0, -66.0)
P2_Y = (-42.0, -35.0)
CRADLE_Y = (-66.0, -45.0)
FRONT_WALL_Y = (-60.0, P.MOTOR_FRONT_FACE_Y)            # boss register wall (split cradle / cap)
CAP_EAR_Y = (-51.0, -45.0)                                # ears stay out of the 45T layer
BRACKET_COL_Y = (-35.0, -31.0)
BRACKET_ARM_Y = (-31.0, -24.2)
REAR_STOP_Y = (-35.0, -26.8)
REAR_BAR_Y0 = MOTOR_Y0 - (P.MOTOR["body_z"][0] + 1.0) + 0.1   # endcap face (model z=-2.3) + 0.1 play

S1_BRG_R, S1_BRG_F = (-70.0, -66.0), (-42.0, -38.0)       # 623
S2_BRG_R, S2_BRG_F = (-71.0, -66.0), (-42.0, -37.0)       # 625
S3_BRG_R, S3_BRG_F = (-55.0, -50.0), (-31.0, -27.0)       # 625 rear, MR105 front (stays outside r48)
BEARING_MR105 = (5.0, 10.0, 4.0)
S1_ROD = (S1_BRG_R[0], S1_BRG_F[1])                       # -70..-38  (32 mm)
S2_ROD = (S2_BRG_R[0], S2_BRG_F[1])                       # -71..-37  (34 mm)
S3_ROD = (S3_BRG_R[0], S3_BRG_F[1])                       # -55..-26  (29 mm)
S3_POST_Y = (P1_Y[1], S3_BRG_R[1])

S1_PIECE_Y = (-65.0, -43.0)    # 48T / hub / 12T m1.0 / spacer
S2_PIECE_Y = (-57.0, -43.0)    # 45T / hub / 12T m1.25
S2_SPACER_Y = (-65.0, -57.0)   # separate printed tube (keeps the 45T printable face-down)
S3_PIECE_Y = (-49.0, -31.0)    # 48T / hub / 12T m1.5 / inner-race boss

# motor can (global XZ section, axis at M)
CAN_R = P.MOTOR["can_d"] / 2          # 10.05
CAN_F = P.MOTOR["can_flats"] / 2      # 7.55
CAN_CLR = 0.15
SHELL = 2.4

POSTS = [(306.5, 72.0), (390.0, 40.0), (330.0, 100.0)]   # P1 -> P2 spacer posts
POST_R = 4.5
PED_HALF_W = 19.5
CAP_SCREW_DX = 15.5
CAP_SCREW_Y = -48.0
CAP_SPLIT_GAP = 0.3
REAR_SCREW_DX, REAR_SCREW_DZ = 14.5, -3.0
LIP_RECESS = 0.5     # inner-race clearance recess behind each bearing; rod ends bear on the lip
PUSH_D = 2.5         # push-out hole through each lip (smaller than the rods)

# S3 bracket local axes in XZ: u away from take-up axis, w perpendicular
_ang = atan2(S3Z - TZ, S3X - TX)
EU = (cos(_ang), sin(_ang))
EW = (-sin(_ang), cos(_ang))
BRK_R = 8.0
BRK_U_END = 22.0
BRK_COL_U = (12.0, 22.0)
BRK_COL_W = 10.0
BRK_SCREWS_UW = [(17.0, -5.5), (17.0, 5.5)]
FG_KEEP_R = P.FINAL_GEAR_ENVELOPE["r"] + 0.3               # trim radius round take-up axis

GRID_P1 = [(x, -65.0) for x in (305.0, 325.0, 345.0, 365.0, 385.0)]
GRID_P2 = [(305.0, -45.0), (365.0, -45.0), (385.0, -45.0)]
for _x, _y in GRID_P1 + GRID_P2:
    assert P.on_grid(_x, _y)

M3 = FASTENERS["M3"]


# ---------------------------------------------------------------- helpers
def _y_prism(sketch_wp, y0, y1):
    """sketch_wp: Workplane in XY where x = global X, y = global Z (not yet extruded).
    Returns a solid spanning global Y y0..y1."""
    return (sketch_wp.extrude(y1 - y0)
            .rotate((0, 0, 0), (1, 0, 0), 90)
            .translate((0, y1, 0)))


def ycyl(x, z, r, y0, y1):
    return _y_prism(cq.Workplane("XY").center(x, z).circle(r), y0, y1)


def ypoly(pts, y0, y1):
    return _y_prism(cq.Workplane("XY").polyline(pts).close(), y0, y1)


def zcyl(x, y, r, z0, z1):
    return cq.Workplane("XY").workplane(offset=z0).center(x, y).circle(r).extrude(z1 - z0)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def double_d(x, z, r, f, y0, y1):
    """Can-shaped prism: circle r clipped to |dz| <= f."""
    c = ycyl(x, z, r, y0, y1)
    return c.intersect(box(x - r - 1, x + r + 1, y0 - 1, y1 + 1, z - f, z + f))


def circle_pts(cx, cz, r, n=48):
    return [(cx + r * cos(2 * pi * i / n), cz + r * sin(2 * pi * i / n)) for i in range(n)]


def hull(points):
    pts = sorted(set((round(x, 4), round(y, 4)) for x, y in points))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def uw(u, w):
    return (S3X + u * EU[0] + w * EW[0], S3Z + u * EU[1] + w * EW[1])


def on_bed(shape_wp, rx=0.0, ry=0.0):
    """print_loc that applies rotation then drops the part onto the bed at the origin."""
    r = loc(rx=rx, ry=ry)
    bb = shape_wp.val().moved(r).BoundingBox()
    return loc(-bb.xmin, -bb.ymin, -bb.zmin) * r


def insert_up(x, y, z0=0.0):
    d, depth = M3["insert"]
    return zcyl(x, y, d / 2, z0, z0 + depth)


def pocket(b):
    return (b[1] + 2 * P.PRESS_FIT) / 2


def bearing_seat(x, z, b, yy, open_to, lip_end):
    """Cut for a bearing pocket (yy), a 0.5 mm inner-race recess and a push-out hole
    through the lip to lip_end. open_to = -1 (pocket opens toward -Y) or +1."""
    clear_r = {(3.0, 10.0): 3.25, (5.0, 10.0): 4.6}.get((b[0], b[1]), 5.5)
    cuts = [ycyl(x, z, pocket(b), yy[0] - 0.01, yy[1] + 0.01)]
    if open_to > 0:   # lip on the -Y side
        cuts.append(ycyl(x, z, clear_r, yy[0] - LIP_RECESS, yy[0] + 0.01))
        cuts.append(ycyl(x, z, PUSH_D / 2, lip_end - 1, yy[0]))
    else:
        cuts.append(ycyl(x, z, clear_r, yy[1] - 0.01, yy[1] + LIP_RECESS))
        cuts.append(ycyl(x, z, PUSH_D / 2, yy[1], lip_end + 1))
    return cuts


# ---------------------------------------------------------------- gears
def gear_layer(m, z, y0, y1, rot_deg):
    g = cq.Workplane("XY").polyline(gear_profile(m, z)).close().extrude(y1 - y0)
    return g.rotate((0, 0, 0), (0, 0, 1), rot_deg).translate((0, 0, y0))


def disc(r, y0, y1):
    return cq.Workplane("XY").workplane(offset=y0).circle(r).extrude(y1 - y0)


def mesh_angles(a, b, z_b):
    """Local rotations (deg) for driver on a (tooth toward b) and gear on b (gap toward a).
    axis_y maps a local angle alpha to global XZ angle -alpha."""
    phi = degrees(atan2(b[1] - a[1], b[0] - a[0]))
    return -phi, -(phi + 180.0) + 180.0 / z_b


A1P, A1G = mesh_angles(P.MOTOR_AXIS, P.SHAFT_S1, ST1["zg"])
A2P, A2G = mesh_angles(P.SHAFT_S1, P.SHAFT_S2, ST2["zg"])
A3P, A3G = mesh_angles(P.SHAFT_S2, P.SHAFT_S3, ST3["zg"])
A4P, A4G = mesh_angles(P.SHAFT_S3, P.TAKEUP_AXIS, ST4["zg"])   # A4G: for the take-up 60T


def motor_pinion():
    y0, y1 = ST1["y"]
    L = y1 - y0
    g = gear_layer(ST1["m"], ST1["zp"], 0, L, A1P)
    g = g.cut(cq.Workplane("XY").circle(1.05).extrude(L))           # 2.1 bore
    for k in range(3):                                               # crush ribs -> ~1.86 eff.
        rib = (cq.Workplane("XY").center(1.03, 0).rect(0.2, 0.4).extrude(L)
               .rotate((0, 0, 0), (0, 0, 1), 120 * k + 60))
        g = g.union(rib)
    return g


def _piece(layers, y_range, rod_d):
    z0 = y_range[0]
    g = None
    for kind, args in layers:
        if kind == "gear":
            m, z, ya, yb, rot = args
            s = gear_layer(m, z, ya - z0, yb - z0, rot)
        else:
            r, ya, yb = args
            s = disc(r, ya - z0, yb - z0)
        g = s if g is None else g.union(s)
    L = y_range[1] - z0
    return g.cut(d_shaft_bore(rod_d, flat_depth=0.5, length=L + 2).translate((0, 0, -1)))


def compound_s1():
    # 48T (bed face) / hub / 12T m1.0 (6 long, 1 mm past the 45T) / spacer to the front 623
    return _piece([("gear", (ST1["m"], ST1["zg"], -65.0, -61.0, A1G)),
                   ("disc", (6.0, -61.0, -58.0)),
                   ("gear", (ST2["m"], ST2["zp"], -58.0, -52.0, A2P)),
                   ("disc", (4.0, -52.0, -43.0))], S1_PIECE_Y, P.ROD_3)


def compound_s2():
    # 45T (bed face) / hub / 12T m1.25 (7 long, 1 mm past the s3 48T)
    return _piece([("gear", (ST2["m"], ST2["zg"], -57.0, -52.0, A2G)),
                   ("disc", (7.0, -52.0, -50.0)),
                   ("gear", (ST3["m"], ST3["zp"], -50.0, -43.0, A3P))], S2_PIECE_Y, P.ROD_5)


def compound_s3():
    # 48T (bed face) / hub / 12T m1.5 (9 long, 1 mm past the 60T)
    return _piece([("gear", (ST3["m"], ST3["zg"], -49.0, -43.0, A3G)),
                   ("disc", (7.0, -43.0, -41.0)),
                   ("gear", (ST4["m"], ST4["zp"], -41.0, -32.0, A4P)),
                   ("disc", (3.5, -32.0, -31.0))], S3_PIECE_Y, P.ROD_5)


def spacer_s2():
    L = S2_SPACER_Y[1] - S2_SPACER_Y[0]
    return cq.Workplane("XY").circle(6.0).circle(2.65).extrude(L)


# ---------------------------------------------------------------- vitamins
def d_rod(d, length, flat=0.5):
    c = cq.Workplane("XY").circle(d / 2).extrude(length)
    keep = cq.Workplane("XY").center(-flat / 2, 0).rect(d - flat, d + 1).extrude(length)
    return c.intersect(keep)


def bearing(b):
    bore, od, w = b
    return cq.Workplane("XY").circle(pocket(b)).circle(bore / 2).extrude(w)   # seated OD


def washer(d_in, d_out, t):
    return cq.Workplane("XY").circle(d_out / 2).circle(d_in / 2).extrude(t)


def motor_shape():
    return cq.importers.importStep(str(P.__file__).replace("params.py", P.MOTOR_STEP))


# ---------------------------------------------------------------- housing
def frame():
    """P1 rear plate + motor cradle + S3 rear post + spacer posts (one print)."""
    pts = [(302, 0), (398, 0), (398, 8), (302, 8)]
    pts += circle_pts(S1X, S1Z, 8.0) + circle_pts(S2X, S2Z, 11.0) + circle_pts(S3X, S3Z, 11.0)
    for x, z in POSTS:
        pts += circle_pts(x, z, 6.5)
    body = ypoly(hull(pts), *P1_Y)
    body = body.union(box(302, 398, P1_Y[0], -60.0, 0, 8))                    # foot flange
    body = body.union(box(MX - PED_HALF_W, MX + PED_HALF_W, CRADLE_Y[0], CRADLE_Y[1], 0, MZ))
    body = body.union(ycyl(S3X, S3Z, 11.0, *S3_POST_Y))
    for x, z in POSTS:
        body = body.union(ycyl(x, z, POST_R, P1_Y[1], P2_Y[0]))

    cut = []
    cut += bearing_seat(S1X, S1Z, P.BEARING_623, S1_BRG_R, +1, P1_Y[0])
    cut += bearing_seat(S2X, S2Z, P.BEARING_625, S2_BRG_R, +1, P1_Y[0])
    cut += bearing_seat(S3X, S3Z, P.BEARING_625, S3_BRG_R, +1, P1_Y[0])
    # motor shaft clearance through P1, pinion pocket in the cradle
    cut.append(ycyl(MX, MZ, 2.5, P1_Y[0] - 1, P1_Y[1] + 0.01))
    cut.append(ycyl(MX, MZ, 6.6, P1_Y[1] - 0.01, FRONT_WALL_Y[0]))
    # can channel + boss register (lower halves; the cap provides the upper halves)
    cut.append(double_d(MX, MZ, CAN_R + CAN_CLR, CAN_F + 0.1, FRONT_WALL_Y[1], CRADLE_Y[1] + 0.01))
    cut.append(ycyl(MX, MZ, P.MOTOR["boss_d"] / 2 + 0.1, FRONT_WALL_Y[0] - 0.01, FRONT_WALL_Y[1] + 0.01))
    d, depth = M3["insert"]
    for sx in (-1, 1):
        cut.append(zcyl(MX + sx * CAP_SCREW_DX, CAP_SCREW_Y, d / 2, MZ - depth, MZ + 1))
    for x, z in POSTS:
        cut.append(ycyl(x, z, d / 2, P2_Y[0] - depth, P2_Y[0] + 1))
    for x, y in GRID_P1:
        cut.append(insert_up(x, y, -0.01))
    for c in cut:
        body = body.cut(c)
    return body


def motor_cap():
    zb = MZ + CAP_SPLIT_GAP
    top = MZ + CAN_F + SHELL
    shell = double_d(MX, MZ, CAN_R + CAN_CLR + SHELL, CAN_F + SHELL, FRONT_WALL_Y[0], CRADLE_Y[1])
    ears = box(MX - PED_HALF_W, MX + PED_HALF_W, CAP_EAR_Y[0], CAP_EAR_Y[1], zb, MZ + 5.5)
    cap = shell.union(ears).intersect(box(MX - 30, MX + 30, -80, 0, zb, top + 1))
    cap = cap.cut(double_d(MX, MZ, CAN_R + CAN_CLR, CAN_F, FRONT_WALL_Y[1], CRADLE_Y[1] + 1))  # clamps top flat
    cap = cap.cut(ycyl(MX, MZ, P.MOTOR["boss_d"] / 2 + 0.1, FRONT_WALL_Y[0] - 1, FRONT_WALL_Y[1] + 0.01))
    for sx in (-1, 1):
        x = MX + sx * CAP_SCREW_DX
        cap = cap.cut(zcyl(x, CAP_SCREW_Y, M3["clear"] / 2, zb - 1, top + 1))
        cap = cap.cut(zcyl(x, CAP_SCREW_Y, (M3["head"] + 0.6) / 2, zb + 2.0, top + 1))
    return cap


def front_plate():
    """P2: S1 + S2 front bearings, keyed collar round the can, carries S3 bracket + rear stop."""
    col = [uw(u, w) for u in BRK_COL_U for w in (-BRK_COL_W, BRK_COL_W)]
    pts = [(300, 0), (396, 0), (396, 8), (300, 8)] + col
    pts += circle_pts(S1X, S1Z, 8.0) + circle_pts(S2X, S2Z, 11.0)
    for x, z in POSTS:
        pts += circle_pts(x, z, 6.5)
    plate = ypoly(hull(pts), *P2_Y)
    # feet flanges toward -Y (stay clear of the cradle pedestal)
    plate = plate.union(box(300, MX - PED_HALF_W - 1.0, -48.0, P2_Y[1], 0, 8))
    plate = plate.union(box(MX + PED_HALF_W + 1.0, 396, -48.0, P2_Y[1], 0, 8))

    cut = []
    cut.append(ycyl(TX, TZ, FG_KEEP_R + 1.0, P2_Y[0] - 10, P2_Y[1] + 1))    # final-gear keep-out
    cut.append(ycyl(S3X, S3Z, 12.5, P2_Y[0] - 10, P2_Y[1] + 1))             # s4 pinion / hub
    cut.append(double_d(MX, MZ, CAN_R + CAN_CLR, CAN_F + 0.1, P2_Y[0] - 1, P2_Y[1] + 1))
    cut.append(box(MX - 8.6, MX + 8.6, P2_Y[0] - 10, P2_Y[1] + 1, -1, MZ))  # terminal/wire slot
    cut += bearing_seat(S1X, S1Z, P.BEARING_623, S1_BRG_F, -1, P2_Y[1])
    cut += bearing_seat(S2X, S2Z, P.BEARING_625, S2_BRG_F, -1, P2_Y[1])
    for x, z in POSTS:
        cut.append(ycyl(x, z, M3["clear"] / 2, P2_Y[0] - 1, P2_Y[1] + 1))
    d, depth = M3["insert"]
    ins = [uw(u, w) for u, w in BRK_SCREWS_UW]
    ins += [(MX + sx * REAR_SCREW_DX, MZ + REAR_SCREW_DZ) for sx in (-1, 1)]
    for x, z in ins:
        cut.append(ycyl(x, z, d / 2, P2_Y[1] - depth, P2_Y[1] + 1))
    for x, y in GRID_P2:
        cut.append(insert_up(x, y, -0.01))
    for c in cut:
        plate = plate.cut(c)
    return plate


def s3_bracket():
    """Holds the S3 front 625 at Y -31..-26; column lands on P2 outside the final-gear envelope."""
    arm = ycyl(S3X, S3Z, BRK_R, *BRACKET_ARM_Y)
    arm = arm.union(ypoly([uw(0, -BRK_R), uw(BRK_U_END, -BRK_R), uw(BRK_U_END, BRK_R), uw(0, BRK_R)],
                          *BRACKET_ARM_Y))
    colm = ypoly([uw(BRK_COL_U[0], -BRK_COL_W), uw(BRK_COL_U[1], -BRK_COL_W),
                  uw(BRK_COL_U[1], BRK_COL_W), uw(BRK_COL_U[0], BRK_COL_W)], *BRACKET_COL_Y)
    b = arm.union(colm)
    b = b.cut(ycyl(TX, TZ, FG_KEEP_R, BRACKET_COL_Y[0] - 1, BRACKET_ARM_Y[1] + 1))
    for c in bearing_seat(S3X, S3Z, BEARING_MR105, S3_BRG_F, -1, BRACKET_ARM_Y[1]):
        b = b.cut(c)
    for u, w in BRK_SCREWS_UW:
        x, z = uw(u, w)
        b = b.cut(ycyl(x, z, M3["clear"] / 2, BRACKET_COL_Y[0] - 1, BRACKET_ARM_Y[1] + 1))
        b = b.cut(ycyl(x, z, (M3["head"] + 0.6) / 2, BRACKET_ARM_Y[1] - 3.0, BRACKET_ARM_Y[1] + 1))
    return b


def rear_stop():
    """Two cheeks screwed to P2's +Y face and a bar behind the rear endcap (positive +Y stop).
    Wires leave the bottom terminals downward between the cheeks, in front of the bar."""
    z0, z1 = MZ - 10.0, MZ + 6.0
    xi, xo = CAN_R + 2.0, CAN_R + 7.0
    s = box(MX - xo, MX - xi, REAR_STOP_Y[0], REAR_STOP_Y[1], z0, z1)
    s = s.union(box(MX + xi, MX + xo, REAR_STOP_Y[0], REAR_STOP_Y[1], z0, z1))
    s = s.union(box(MX - xo, MX + xo, REAR_BAR_Y0, REAR_STOP_Y[1], z0, z1))
    s = s.cut(ycyl(MX, MZ, 1.6, REAR_BAR_Y0 - 1, REAR_STOP_Y[1] + 1))     # rear bushing
    for sx in (-1, 1):
        x, z = MX + sx * REAR_SCREW_DX, MZ + REAR_SCREW_DZ
        s = s.cut(ycyl(x, z, M3["clear"] / 2, REAR_STOP_Y[0] - 1, REAR_STOP_Y[1] + 1))
        s = s.cut(ycyl(x, z, (M3["head"] + 0.6) / 2, REAR_STOP_Y[1] - 3.0, REAR_STOP_Y[1] + 1))
    return s


# ---------------------------------------------------------------- build
def build():
    parts = []
    I = cq.Location()
    GRAY, WHITE, METAL = "gray40", "white", "gray80"

    def printed(name, shape, rx, notes, l=I, color=GRAY):
        parts.append(Part(name, shape, l, print_loc=on_bed(shape, rx=rx), color=color, notes=notes))

    printed("gearbox_frame", frame(), 90,
            "Print on rear face (Y-74 down), no supports. 10x M3 inserts (5 base, 2 cap, 3 posts). "
            "623 + 2x 625 pockets press fit.")
    printed("gearbox_motor_cap", motor_cap(), 180,
            "Print top-down. 2x M3x8 into frame inserts; clamps can top flat + upper half of the boss.")
    printed("gearbox_front_plate", front_plate(), -90,
            "Print on +Y face. Double-D collar keys the can; 623 + 625 pockets; 3x M3 to frame posts; "
            "3 base inserts, 2 bracket inserts, 2 rear-stop inserts.")
    printed("gearbox_s3_bracket", s3_bracket(), -90, "Print on +Y (lip) face. 2x M3x14 into P2 inserts.")
    printed("gearbox_motor_rear_stop", rear_stop(), -90,
            "Print on +Y face. 2x M3x12 into P2 inserts; bar sits 0.1 mm behind the rear endcap.")

    parts.append(Part("gearbox_pinion_s1", motor_pinion(), axis_y(MX, MZ, ST1["y"][0]), color=WHITE,
                      notes="12T m0.8, 100% infill. 2.1 bore + 3 crush ribs (~1.86 eff.) press fit on the "
                            "2.0 motor shaft; support the motor rear when pressing."))
    parts.append(Part("gearbox_gear_s1", compound_s1(), axis_y(S1X, S1Z, S1_PIECE_Y[0]), color=WHITE,
                      notes="48T m0.8 + 12T m1.0 + spacer, D-bore 3 mm. Print 48T face down."))
    parts.append(Part("gearbox_gear_s2", compound_s2(), axis_y(S2X, S2Z, S2_PIECE_Y[0]), color=WHITE,
                      notes="45T m1.0 + 12T m1.25, D-bore 5 mm. Print 45T face down."))
    parts.append(Part("gearbox_spacer_s2", spacer_s2(), axis_y(S2X, S2Z, S2_SPACER_Y[0]), color=WHITE,
                      notes="Tube OD12/ID5.3 x 8, between the rear S2 washer and the 45T."))
    parts.append(Part("gearbox_gear_s3", compound_s3(), axis_y(S3X, S3Z, S3_PIECE_Y[0]), color=WHITE,
                      notes="48T m1.25 + 12T m1.5, D-bore 5 mm. Print 48T face down."))

    # ---- vitamins
    parts.append(Part("gearbox_motor", motor_shape(), loc(MX, MOTOR_Y0, MZ, rx=90, rz=180),
                      printed=False, color=METAL,
                      notes="130 motor (<=6 V), shaft -Y, terminals rolled to the bottom (wire exit down)."))
    for nm, (x, z), d, yy in [("gearbox_rod_s1", P.SHAFT_S1, P.ROD_3, S1_ROD),
                              ("gearbox_rod_s2", P.SHAFT_S2, P.ROD_5, S2_ROD),
                              ("gearbox_rod_s3", P.SHAFT_S3, P.ROD_5, S3_ROD)]:
        parts.append(Part(nm, d_rod(d, yy[1] - yy[0]), axis_y(x, z, yy[0]), printed=False, color=METAL,
                          notes=f"{d:.0f} mm D-shaft, {yy[1] - yy[0]:.0f} mm"))
    for nm, (x, z), b, yy in [("gearbox_bearing_s1_rear", P.SHAFT_S1, P.BEARING_623, S1_BRG_R),
                              ("gearbox_bearing_s1_front", P.SHAFT_S1, P.BEARING_623, S1_BRG_F),
                              ("gearbox_bearing_s2_rear", P.SHAFT_S2, P.BEARING_625, S2_BRG_R),
                              ("gearbox_bearing_s2_front", P.SHAFT_S2, P.BEARING_625, S2_BRG_F),
                              ("gearbox_bearing_s3_rear", P.SHAFT_S3, P.BEARING_625, S3_BRG_R),
                              ("gearbox_bearing_s3_front", P.SHAFT_S3, BEARING_MR105, S3_BRG_F)]:
        parts.append(Part(nm, bearing(b), axis_y(x, z, yy[0]), printed=False, color=METAL,
                          notes="modelled at seated (pocket) OD"))
    m3w, m5w = (3.2, 7.0, 1.0), (5.3, 10.0, 1.0)
    for nm, (x, z), w, y0 in [("gearbox_washer_s1_rear", P.SHAFT_S1, m3w, -66.0),
                              ("gearbox_washer_s1_front", P.SHAFT_S1, m3w, -43.0),
                              ("gearbox_washer_s2_rear", P.SHAFT_S2, m5w, -66.0),
                              ("gearbox_washer_s2_front", P.SHAFT_S2, m5w, -43.0),
                              ("gearbox_washer_s3_rear", P.SHAFT_S3, m5w, -50.0)]:
        parts.append(Part(nm, washer(*w), axis_y(x, z, y0), printed=False, color=METAL,
                          notes="DIN125 washer stack, 1.0 mm"))
    return parts


if __name__ == "__main__":
    export(build(), "gearbox")

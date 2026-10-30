"""WS-E: filament station = inlet guide + sponge wiper + auto-stop + outlet.

Everything is modelled directly in GLOBAL coordinates (loc = identity) inside
params.FILAMENT_STATION_BOX (X 222..268, Y 0..90, Z 0..165).

Filament path (axis Z = PATH_Z = 140, Y = SPOOL_CENTER_Y = 45), flowing +X:
  X 227      inlet: two 5 mm steel rods (axis Y) above/below the path, cantilevered
             from a -Y cheek, open +Y end -> filament slides in sideways.
  X 233..252 wiper: lower + upper sponge pad in an open-top pocket; the end walls
             carry radiused V-notches (Y-centering, bottom = FIL_SUPPORT_Z).
             Upper pad is impaled on 4 spikes under a gravity lid (ballast hole).
  X 256      auto-stop: KW11-3Z roller-lever switch on the +Y side, lever along +X;
             a printed shoe caps the roller and reaches under the filament. Filament
             tension holds the shoe down (switch actuated = closed, NO wiring).
             Tail passes -> switch spring lifts the shoe -> contact opens -> motor stops.
  X 264      outlet: two printed rollers on vertical M3 screws, 3.8 mm gap, let the
             downstream filament swing to the traverse eyelet (Y 10..85).
Station body is one print (upright, as installed) bolted with 4x M3 on grid X=245.
"""
from math import sqrt

import cadquery as cq
from cadquery import Vector, Solid

import params as P
from lib.common import FASTENERS
from lib.part import Part, loc, export

PZ = P.PATH_Z                      # 140 filament axis
CY = P.SPOOL_CENTER_Y              # 45
FIL_SUPPORT_Z = PZ - P.FILAMENT_D_MAX / 2   # 138.575: underside of 2.85 filament

# ---------------------------------------------------------------- layout
FOOT_Z = 8.0
COL = dict(x=(224.0, 266.0), y=(26.0, 64.0), wall=3.0)
DECK = dict(x=(222.0, 268.0), y=(24.0, 64.0), z=(112.0, 116.0))
DECK_TOP = DECK["z"][1]
FOOT_SCREWS = [(245.0, 15.0), (245.0, 35.0), (245.0, 55.0), (245.0, 75.0)]
WIRE_HOLE = (235.0, 45.0, 8.0)     # X, Y, d through the foot -> base groove

# inlet
ROD_D = P.ROD_5
INLET_X = 227.0
ROD_LO_Z = FIL_SUPPORT_Z - ROD_D / 2          # top of lower rod = support line
ROD_HI_Z = FIL_SUPPORT_Z + 4.9 + ROD_D / 2    # 4.9 mm gap between rods
ROD_Y = (24.0, 64.0)
CHEEK = dict(x=(222.0, 232.0), y=(24.0, 32.0), z=(DECK_TOP, 151.0))

# wiper holder
POCKET_X = (235.4, 249.6)
POCKET_Y = (38.0, 52.0)
FLOOR_Z = (125.6, 128.0)
WALL_TOP = 155.0
HOLDER_X = (233.0, 252.0)
HOLDER_Y = (35.6, 54.4)
PAD = (14.0, 13.8, 12.0)           # X, Y, Z sponge pad
LID_Z = (152.0, 162.0)

# switch (KW11-3Z): local frame x=length, y=thickness, z=height; placed by translation
SW = P.MICROSWITCH
SW_L, SW_T, SW_H = SW["body"]
SW_ORG = (240.1, 56.0 + SW_T / 2, 121.5)   # body x0, centre Y, bottom Z
SW_HOLE_Z = 2.9                             # hole height above body bottom (check your switch)
SW_ROLLER_X = 16.0                          # roller centre from pivot end
SW_ROLLER_R = 2.4
SW_ROLLER_W = 3.0
SW_ROLLER_ZC = SW_H + 0.4 + SW_ROLLER_R     # actuated position
SHOE_TOP = FIL_SUPPORT_Z + 0.9              # lifts filament 0.9 mm when actuated
WING = dict(x=(236.0, 260.0), y=(52.0, 56.0), z=(DECK_TOP, 131.0))

# outlet rollers
ROLLER_OD, ROLLER_ID = 8.1, 3.3
ROLLER_X = 264.2
ROLLER_GAP = 3.8
ROLLER_Y = (CY - (ROLLER_GAP + ROLLER_OD) / 2, CY + (ROLLER_GAP + ROLLER_OD) / 2)
ROLLER_Z = (133.5, 147.0)
RBLOCK = dict(x=(260.5, 268.0), y=(33.0, 57.0), z=(DECK_TOP, 133.0))


# ---------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    return Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, Vector(x0, y0, z0))


def zcyl(d, x, y, z0, z1):
    return Solid.makeCylinder(d / 2, z1 - z0, Vector(x, y, z0), Vector(0, 0, 1))


def ycyl(d, x, z, y0, y1):
    return Solid.makeCylinder(d / 2, y1 - y0, Vector(x, y0, z), Vector(0, 1, 0))


def xcyl(d, y, z, x0, x1):
    return Solid.makeCylinder(d / 2, x1 - x0, Vector(x0, y, z), Vector(1, 0, 0))


def yz_prism(pts, x0, x1):
    """Polygon in the YZ plane (points (y, z)) extruded from x0 to x1."""
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts).close().extrude(x1 - x0).val()


def fuse_all(shapes):
    s = shapes[0]
    for t in shapes[1:]:
        s = s.fuse(t)
    return s.clean()


def cut_all(s, cuts):
    for c in cuts:
        s = s.cut(c)
    return s.clean()


def diamond_y(xc, zc, half, y0, y1):
    """45-degree diamond window (axis Y) - self-supporting."""
    a = half * sqrt(2)
    b = Solid.makeBox(a, y1 - y0, a, Vector(-a / 2, y0, -a / 2))
    return b.rotate(Vector(0, 0, 0), Vector(0, 1, 0), 45).translate(Vector(xc, 0, zc))


def diamond_x(yc, zc, half, x0, x1):
    a = half * sqrt(2)
    b = Solid.makeBox(x1 - x0, a, a, Vector(x0, -a / 2, -a / 2))
    return b.rotate(Vector(0, 0, 0), Vector(1, 0, 0), 45).translate(Vector(0, yc, zc))


def try_fillet(shape, r, sel):
    try:
        wp = cq.Workplane().add(shape).edges(sel).fillet(r)
        return wp.val()
    except Exception:
        return shape


def wp(s):
    return cq.Workplane("XY").add(s)


def bed_loc(shape, flip=False):
    bb = shape.BoundingBox()
    cx, cy = (bb.xmin + bb.xmax) / 2, (bb.ymin + bb.ymax) / 2
    if flip:   # rx=180: (x,y,z) -> (x,-y,-z), then translate
        return loc(-cx, cy, bb.zmax, rx=180)
    return loc(-cx, -cy, -bb.zmin)


# ---------------------------------------------------------------- station body
def notch_wall(x0, x1):
    """End wall of the pad pocket with an open-top, radiused V notch at Y=CY."""
    w = box(x0, x1, HOLDER_Y[0], HOLDER_Y[1], DECK_TOP, WALL_TOP)
    r = 1.6                                   # notch bottom radius (passes 2.85)
    zc = FIL_SUPPORT_Z + r
    top = WALL_TOP + 1
    v = yz_prism([(CY - r, zc), (CY - 6.5, top), (CY + 6.5, top), (CY + r, zc)], x0 - 1, x1 + 1)
    v = v.fuse(xcyl(2 * r, CY, zc, x0 - 1, x1 + 1))
    w = w.cut(v).clean()
    # round the notch edges on both X faces (filament arrives at up to ~19 deg in Y)
    sel = cq.selectors.BoxSelector((x0 - 0.1, CY - 7, FIL_SUPPORT_Z - 0.5),
                                   (x1 + 0.1, CY + 7, WALL_TOP - 0.5))
    return try_fillet(w, 0.9, sel)


def station_body():
    x0, x1 = COL["x"]; y0, y1 = COL["y"]; t = COL["wall"]
    parts = []
    # foot
    parts.append(box(222.0, 268.0, 5.0, 85.0, 0.0, FOOT_Z))
    # hollow column (wire chase) + deck
    col = box(x0, x1, y0, y1, FOOT_Z, DECK["z"][0])
    col = col.cut(box(x0 + t, x1 - t, y0 + t, y1 - t, FOOT_Z, DECK["z"][0]))
    parts.append(col)
    parts.append(box(*DECK["x"], *DECK["y"], *DECK["z"]))
    # 45-degree gussets foot <-> column
    for gx in (229.0, 258.0):
        parts.append(yz_prism([(y0, FOOT_Z), (y0 - 18, FOOT_Z), (y0, FOOT_Z + 18)], gx, gx + 3))
        parts.append(yz_prism([(y1, FOOT_Z), (y1 + 18, FOOT_Z), (y1, FOOT_Z + 18)], gx, gx + 3))
    # inlet cheek
    parts.append(box(*CHEEK["x"], *CHEEK["y"], *CHEEK["z"]))
    # wiper holder
    parts.append(box(HOLDER_X[0], HOLDER_X[1], HOLDER_Y[0], POCKET_Y[0], FLOOR_Z[0], WALL_TOP))   # -Y wall (open below floor -> dust exit)
    parts.append(box(HOLDER_X[0], HOLDER_X[1], POCKET_Y[1], HOLDER_Y[1], DECK_TOP, WALL_TOP))    # +Y wall
    parts.append(notch_wall(HOLDER_X[0], POCKET_X[0]))
    parts.append(notch_wall(POCKET_X[1], HOLDER_X[1]))
    floor = box(*POCKET_X, *POCKET_Y, *FLOOR_Z)
    for sy in (40.5, 44.25, 48.0):                                 # dust slots
        floor = floor.cut(box(237.0, 248.0, sy, sy + 1.5, FLOOR_Z[0] - 1, FLOOR_Z[1] + 1))
    parts.append(floor)
    # switch bracket wing
    parts.append(box(*WING["x"], *WING["y"], *WING["z"]))
    # outlet roller block
    parts.append(box(*RBLOCK["x"], *RBLOCK["y"], *RBLOCK["z"]))

    body = fuse_all(parts)

    cuts = []
    # M3 heat-set inserts from below on grid points
    di, depth = FASTENERS["M3"]["insert"]
    for gx, gy in FOOT_SCREWS:
        assert P.on_grid(gx, gy)
        cuts.append(zcyl(di, gx, gy, -1.0, depth))
    # wire path: deck slot under the switch terminals -> column -> foot hole
    cuts.append(box(242.0, 258.0, 56.5, 61.0, DECK["z"][0] - 1, DECK_TOP + 0.01))
    cuts.append(zcyl(WIRE_HOLE[2], WIRE_HOLE[0], WIRE_HOLE[1], -1.0, FOOT_Z + 1))
    # column windows (45 deg diamonds, wire access + weight)
    for zc in (42.0, 82.0):
        cuts.append(diamond_y(245.0, zc, 14.0, y0 - 1, y1 + 1))
        cuts.append(diamond_x(45.0, zc, 11.0, x0 - 1, x1 + 1))
    # inlet rod holes (5 mm steel rods, press fit)
    for z in (ROD_LO_Z, ROD_HI_Z):
        cuts.append(ycyl(ROD_D, INLET_X, z, CHEEK["y"][0] - 1, CHEEK["y"][1] + 1))
    # switch pilot holes (M2 thread-forming, drill 1.7 on print; modelled 2.0)
    for hx in (5.25, 5.25 + SW["hole_spacing"]):
        cuts.append(ycyl(2.0, SW_ORG[0] + hx, SW_ORG[2] + SW_HOLE_Z, WING["y"][0] - 0.01, WING["y"][1] + 0.01))
    # outlet roller axles: M3 thread-forming pilot (2.5 on print; modelled 3.0)
    for ry in ROLLER_Y:
        cuts.append(zcyl(3.0, ROLLER_X, ry, RBLOCK["z"][1] - 12, RBLOCK["z"][1] + 0.01))
    return cut_all(body, cuts)


# ---------------------------------------------------------------- lid + pads
def lid():
    cx, cy = (POCKET_X[0] + POCKET_X[1]) / 2, CY
    b = box(POCKET_X[0] + 0.2, POCKET_X[1] - 0.2, POCKET_Y[0] + 0.2, POCKET_Y[1] - 0.2, *LID_Z)
    for sx in (-4, 4):
        for sy in (-4, 4):
            b = b.fuse(zcyl(2.0, cx + sx, cy + sy, LID_Z[0] - 4, LID_Z[0]))
    b = b.cut(zcyl(8.4, cx, cy, LID_Z[1] - 7, LID_Z[1] + 1))     # ballast hole (M8 bolt stub / 8 mm rod)
    return b.clean()


def pad(z0, spikes=False):
    cx = (POCKET_X[0] + POCKET_X[1]) / 2
    s = box(cx - PAD[0] / 2, cx + PAD[0] / 2, CY - PAD[1] / 2, CY + PAD[1] / 2, z0, z0 + PAD[2])
    if spikes:
        for sx in (-4, 4):
            for sy in (-4, 4):
                s = s.cut(zcyl(2.0, cx + sx, CY + sy, LID_Z[0] - 4, LID_Z[0] + 0.1))
    return s


# ---------------------------------------------------------------- switch + shoe
def microswitch():
    x0, yc, z0 = SW_ORG
    t2 = SW_T / 2
    body = box(x0, x0 + SW_L, yc - t2, yc + t2, z0, z0 + SW_H)
    for hx in (5.25, 5.25 + SW["hole_spacing"]):
        body = body.cut(ycyl(SW["hole_d"], x0 + hx, z0 + SW_HOLE_Z, yc - t2 - 1, yc + t2 + 1))
    term = box(x0 + 1.0, x0 + SW_L - 1.0, yc - 0.3, yc + 0.3, z0 - 3.5, z0)
    strip = box(x0 + 0.5, x0 + SW_ROLLER_X - SW_ROLLER_R - 0.2, yc - 1.8, yc + 1.8,
                z0 + SW_H + 0.4, z0 + SW_H + 0.9)
    roller = ycyl(2 * SW_ROLLER_R, x0 + SW_ROLLER_X, z0 + SW_ROLLER_ZC,
                  yc - SW_ROLLER_W / 2, yc + SW_ROLLER_W / 2)
    return fuse_all([body, term, strip, roller])


def stop_shoe():
    x0, yc, z0 = SW_ORG
    rx, rz = x0 + SW_ROLLER_X, z0 + SW_ROLLER_ZC
    cap_x = (rx - 3.8, rx + 3.8)
    cap_z0 = z0 + SW_H + 0.3
    cav_r = SW_ROLLER_R + 0.15
    cap = box(cap_x[0], cap_x[1], yc - 4.0, yc + 4.0, cap_z0, SHOE_TOP)
    arm = box(rx - 3.1, rx + 3.1, CY - 4.0, yc - 4.0 + 0.01, SHOE_TOP - 2.8, SHOE_TOP)
    s = cap.fuse(arm).clean()
    # roller cavity, open at the bottom (cap drops onto the roller from above)
    cav = box(rx - cav_r, rx + cav_r, yc - SW_ROLLER_W / 2 - 0.1, yc + SW_ROLLER_W / 2 + 0.1,
              cap_z0 - 1, rz + cav_r)
    # slot for the lever strip toward the pivot
    slot = box(cap_x[0] - 1, rx, yc - 1.95, yc + 1.95, cap_z0 - 1, z0 + SW_H + 1.05)
    s = s.cut(cav).cut(slot)
    # shallow guide groove for the filament along X at Y=CY
    s = s.cut(xcyl(8.0, CY, SHOE_TOP + 4.0 - 0.5, rx - 10, rx + 10)).clean()
    # round the edges the filament bends over
    s = try_fillet(s, 1.2, cq.selectors.BoxSelector((rx - 4, CY - 4.5, SHOE_TOP - 1.0),
                                                    (rx + 4, CY + 4.5, SHOE_TOP + 0.1)))
    return s


# ---------------------------------------------------------------- outlet rollers
def outlet_roller(y):
    return zcyl(ROLLER_OD, ROLLER_X, y, *ROLLER_Z).cut(zcyl(ROLLER_ID, ROLLER_X, y, ROLLER_Z[0] - 1, ROLLER_Z[1] + 1))


def roller_screw(y):
    head = zcyl(5.5, ROLLER_X, y, ROLLER_Z[1], ROLLER_Z[1] + 2.0)
    shank = zcyl(2.9, ROLLER_X, y, ROLLER_Z[1] - 20.0, ROLLER_Z[1])
    return head.fuse(shank).clean()


# ---------------------------------------------------------------- build
def build():
    out = []
    body = station_body()
    out.append(Part("wiper_station", wp(body), print_loc=bed_loc(body), color="orange",
                    notes="Print upright as installed (foot on bed), 4 walls, 30% gyroid. "
                          "4x M3 heat-set inserts from below at grid (245, 15/35/55/75). "
                          "Bridges: pocket floor (14 mm), deck over column (36x32 mm interior). "
                          "Ream inlet rod holes to 5.0 press fit; M2 pilots 1.7, M3 roller pilots 2.5."))
    l = lid()
    out.append(Part("wiper_lid", wp(l), print_loc=bed_loc(l, flip=True), color="orange",
                    notes="Print top face down (spikes up). Gravity lid; optional ballast in 8.4 hole."))
    sh = stop_shoe()
    out.append(Part("wiper_stop_shoe", wp(sh), print_loc=bed_loc(sh, flip=True), color="red",
                    notes="Print upside down (filament face on bed). Press onto the switch roller; "
                          "secure with a drop of CA if loose. Sand top to tune switch point."))
    for tag, y in zip(("a", "b"), ROLLER_Y):
        r = outlet_roller(y)
        out.append(Part(f"wiper_outlet_roller_{tag}", wp(r), print_loc=bed_loc(r), color="orange",
                        notes="Print upright; ream bore 3.3 so it spins freely on the M3 screw."))
        out.append(Part(f"wiper_roller_screw_{tag}", wp(roller_screw(y)), printed=False, color="gray",
                        notes="M3x20 socket head + M3 washer under roller, thread-forming into 2.5 pilot"))
    out.append(Part("wiper_microswitch", wp(microswitch()), printed=False, color="black",
                    notes="KW11-3Z / SS-5GL roller lever microswitch, use NO+COM, 2x M2x10 thread-forming"))
    out.append(Part("wiper_sponge_lower", wp(pad(FLOOR_Z[1])), printed=False, color="yellow",
                    notes="sponge 14x14x12 mm (no scourer layer)"))
    out.append(Part("wiper_sponge_upper", wp(pad(FLOOR_Z[1] + PAD[2], spikes=True)), printed=False,
                    color="yellow", notes="sponge 14x14x12 mm, pushed onto lid spikes"))
    for tag, z in (("lower", ROD_LO_Z), ("upper", ROD_HI_Z)):
        out.append(Part(f"wiper_inlet_rod_{tag}", wp(ycyl(ROD_D - 0.05, INLET_X, z, *ROD_Y)), printed=False,
                        color="gray", notes="5 mm steel rod x 40 mm, ends chamfered/polished"))
    return out


if __name__ == "__main__":
    export(build(), "wiper")

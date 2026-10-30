"""Base plate halves, joint hardware and rubber feet (WS-F).

Layout (global frame, Z = 0 is the top of the plate):
  - plate Z -6..0, ribs Z -18..-6 (params.BASE_THICK, BASE_RIB_H)
  - ribs run on grid MID-lines (X = 15 + 20k, Y = -75 + 20k) so they never sit
    on a grid hole; spacing 40 mm, leaving open cells for screw/driver access
  - joint: thick blocks either side of X = 210 (half 1: X 198..210,
    half 2: X 210..221), 4 x M3x25 cross bolts (head in half-1 cell, nut in a
    trap on half 2), 4 x Ø5x18 steel dowel pins for coplanarity
  - grid holes: Ø3.4 through, Ø6.2 counterbore from below up to Z -3 (3 mm of
    plate left under the head). Grid column X = 205 lies inside the half-1
    joint block: there the hole is a plain Ø3.4 through the full 18 mm
    (screw head bears on the block underside -> use M3 x 25 there).
  - wiring: Ø10 pass-throughs (electronics box, motor, auto-stop), wires run
    in the open underside cells through 10 x 6 mm notches cut in the rib
    bottoms; short 5 x 2.5 mm grooves on top lead to the motor / station holes
"""
import cadquery as cq

import params as P
from lib.common import FASTENERS
from lib.part import Part, loc

T = P.BASE_THICK                       # 6
ZB = -(P.BASE_THICK + P.BASE_RIB_H)    # -18 rib bottom
RIB_W = 3.0
EDGE_W = 2.4
X0, X1 = P.BASE_X
Y0, Y1 = P.BASE_Y
SPLIT = P.BASE_SPLIT_X

# ribs (centre lines)
XRIBS = {1: [35.0, 75.0, 115.0, 155.0], 2: [255.0, 295.0, 335.0, 375.0]}
YRIBS = [-55.0, -15.0, 25.0, 65.0]
JOINT = {1: (198.0, SPLIT), 2: (SPLIT, 221.0)}

M3 = FASTENERS["M3"]
CB_D = M3["head"] + 0.6                # 6.2 counterbore
CB_TOP = -3.0                          # counterbore floor
JOINT_BOLT_Y = [-75.0, 5.0, 45.0, 85.0]
JOINT_DOWEL_Y = [-55.0, -15.0, 25.0, 65.0]
JOINT_Z = -12.0
DOWEL_D, DOWEL_L, DOWEL_HOLE = 5.0, 18.0, 5.15
NUT_TRAP_DEPTH = 2.6

# wiring pass-throughs (top -> underside), all clear of grid holes and ribs
WIRE_HOLE_D = 10.0
WIRE_HOLES = {
    "ebox": (55.0, -62.0),     # under the electronics box floor
    "motor": (315.0, 5.0),     # just +Y of the gearbox, near the motor endcap
    "autostop": (235.0, 45.0), # under the wiper / auto-stop station foot
}
# rib notches (underside wire route): (x, y, along) along = axis the rib runs on
NOTCH_W, NOTCH_H = 10.0, 6.0
ROUTE_Y = -35.0
NOTCHES_X_RIB = [(x, ROUTE_Y) for x in (75.0, 115.0, 155.0, 255.0, 295.0)]  # cut through X-ribs
NOTCHES_Y_RIB = [(55.0, -55.0), (315.0, -15.0), (235.0, -15.0), (235.0, 25.0)]
# top grooves (x0, y0, x1, y1), axis aligned, 5 wide 2.5 deep, on grid mid-lines
GROOVE_W, GROOVE_D = 5.0, 2.5
TOP_GROOVES = [
    (335.0, -24.0, 335.0, 5.0),    # motor endcap -> motor hole (on mid-line X 335)
    (315.0, 5.0, 335.0, 5.0),
    (235.0, 45.0, 265.0, 45.0),    # auto-stop foot -> station hole
]

# rubber feet (stick-on bumpers Ø10 x 3) on Ø16 pads at cell centres
FOOT_D, FOOT_H, FOOT_PAD_D, FOOT_POCKET = 10.0, 3.0, 16.0, 1.0
FEET = [(15.0, -75.0), (15.0, 85.0), (395.0, -75.0), (395.0, 85.0),
        (235.0, -75.0), (235.0, 85.0)]   # last two support the joint region


def _box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def _cyls(pts, d, z0, z1):
    if not pts:
        return None
    return cq.Workplane("XY").workplane(offset=z0).pushPoints(pts).circle(d / 2).extrude(z1 - z0)


def _cut(body, cutter):
    return body if cutter is None else body.cut(cutter)


def _in(h, x, margin=0.0):
    a, b = (X0, SPLIT) if h == 1 else (SPLIT, X1)
    return a - margin <= x <= b + margin


def half(h):
    xa, xb = (X0, SPLIT) if h == 1 else (SPLIT, X1)
    body = _box(xa, xb, Y0, Y1, -T, 0)
    ribs = []
    # outer edge ribs
    ribs.append(_box(xa, xb, Y0, Y0 + EDGE_W, ZB, -T))
    ribs.append(_box(xa, xb, Y1 - EDGE_W, Y1, ZB, -T))
    if h == 1:
        ribs.append(_box(X0, X0 + EDGE_W, Y0, Y1, ZB, -T))
    else:
        ribs.append(_box(X1 - EDGE_W, X1, Y0, Y1, ZB, -T))
    for x in XRIBS[h]:
        ribs.append(_box(x - RIB_W / 2, x + RIB_W / 2, Y0, Y1, ZB, -T))
    for y in YRIBS:
        ribs.append(_box(xa, xb, y - RIB_W / 2, y + RIB_W / 2, ZB, -T))
    ja, jb = JOINT[h]
    ribs.append(_box(ja, jb, Y0, Y1, ZB, -T))
    feet = [p for p in FEET if _in(h, p[0])]
    ribs.append(_cyls(feet, FOOT_PAD_D, ZB, -T))
    for r in ribs:
        body = body.union(r)

    # --- grid holes
    pts = [p for p in P.grid_points() if _in(h, p[0])]
    body = body.cut(_cyls(pts, M3["clear"], ZB - 1, 1))
    cb = [p for p in pts if not (JOINT[1][0] - CB_D / 2 < p[0] < JOINT[2][1] + CB_D / 2)]
    body = _cut(body, _cyls(cb, CB_D, ZB - 1, CB_TOP))

    # --- wiring
    wh = [p for p in WIRE_HOLES.values() if _in(h, p[0])]
    body = _cut(body, _cyls(wh, WIRE_HOLE_D, ZB - 1, 1))
    for x, y in NOTCHES_X_RIB:
        if _in(h, x):
            body = body.cut(_box(x - 3, x + 3, y - NOTCH_W / 2, y + NOTCH_W / 2, ZB - 1, ZB + NOTCH_H))
    for x, y in NOTCHES_Y_RIB:
        if _in(h, x):
            body = body.cut(_box(x - NOTCH_W / 2, x + NOTCH_W / 2, y - 3, y + 3, ZB - 1, ZB + NOTCH_H))
    # route through both joint blocks
    jy = ROUTE_Y
    body = body.cut(_box(ja - 1, jb + 1, jy - NOTCH_W / 2, jy + NOTCH_W / 2, ZB - 1, ZB + NOTCH_H))
    for gx0, gy0, gx1, gy1 in TOP_GROOVES:
        if _in(h, gx0) and _in(h, gx1):
            body = body.cut(_box(min(gx0, gx1) - GROOVE_W / 2, max(gx0, gx1) + GROOVE_W / 2,
                                 min(gy0, gy1) - GROOVE_W / 2, max(gy0, gy1) + GROOVE_W / 2,
                                 -GROOVE_D, 1))

    # --- feet pockets
    body = _cut(body, _cyls(feet, FOOT_D + 0.5, ZB - 1, ZB + FOOT_POCKET))

    # --- joint: cross bolts, nut traps, dowels
    bolts = cq.Workplane("YZ", origin=(ja - 1, 0, 0)).pushPoints(
        [(y, JOINT_Z) for y in JOINT_BOLT_Y]).circle(M3["clear"] / 2).extrude(jb - ja + 2)
    body = body.cut(bolts)
    dowel_x0 = SPLIT - 10.0 if h == 1 else SPLIT
    dowels = cq.Workplane("YZ", origin=(dowel_x0, 0, 0)).pushPoints(
        [(y, JOINT_Z) for y in JOINT_DOWEL_Y]).circle(DOWEL_HOLE / 2).extrude(10.0)
    body = body.cut(dowels)
    if h == 2:
        traps = cq.Workplane("YZ", origin=(jb - NUT_TRAP_DEPTH, 0, 0)).pushPoints(
            [(y, JOINT_Z) for y in JOINT_BOLT_Y]).polygon(6, (M3["nut_af"] + 0.2) / 0.866025) \
            .extrude(NUT_TRAP_DEPTH + 1)
        body = body.cut(traps)

    # print: plate on the bed, ribs up (no supports)
    pl = loc(-xa, Y1, 0, rx=180)  # flip: top face on bed, ribs up (Z 0..18)
    return Part(
        name=f"base_plate_half{h}", shape=body, loc=loc(), printed=True,
        print_loc=pl, color="gray",
        notes=("Print top face DOWN on the bed (ribs up), no supports. "
               f"{'Supply/electronics side X 0..210.' if h == 1 else 'Take-up/gearbox side X 210..420; heat no inserts, M3 nuts go in the 4 traps on the joint face.'} "
               "Grid screws: M3 x (3 mm plate + insert engagement), head in the Ø6.2 counterbore; "
               "grid column X=205 has no counterbore (inside joint block): M3 x 25."),
    )


def joint_hardware():
    solids = []
    ja, jb = JOINT[1][0], JOINT[2][1]
    for y in JOINT_BOLT_Y:
        head = cq.Solid.makeCylinder(5.5 / 2, 2.95, cq.Vector(ja - 3.0, y, JOINT_Z), cq.Vector(1, 0, 0))
        shank = cq.Solid.makeCylinder(1.5, 25.0, cq.Vector(ja - 0.05, y, JOINT_Z), cq.Vector(1, 0, 0))
        nut = (cq.Workplane("YZ", origin=(jb - NUT_TRAP_DEPTH + 0.1, 0, 0)).center(y, JOINT_Z)
               .polygon(6, M3["nut_af"] / 0.866025).circle(1.5).extrude(2.4).val())
        solids += [head, shank, nut]
    for y in JOINT_DOWEL_Y:
        solids.append(cq.Solid.makeCylinder(DOWEL_D / 2, DOWEL_L, cq.Vector(SPLIT - DOWEL_L / 2, y, JOINT_Z),
                                            cq.Vector(1, 0, 0)))
    return Part(
        name="base_joint_hardware", shape=cq.Workplane().add(cq.Compound.makeCompound(solids)),
        printed=False, color="gray80",
        notes="4x M3x25 socket cap + 4x M3 hex nut (in traps on half 2); "
              "4x Ø5x18 steel dowel pin (cut from 5 mm rod), glue/press into half 1.",
    )


def rubber_feet():
    solids = [cq.Solid.makeCylinder(FOOT_D / 2, FOOT_H, cq.Vector(x, y, ZB + FOOT_POCKET - 0.05 - FOOT_H))
              for x, y in FEET]
    return Part(
        name="base_rubber_feet", shape=cq.Workplane().add(cq.Compound.makeCompound(solids)),
        printed=False, qty=len(FEET), color="black",
        notes="Self-adhesive bumpers Ø10 x 3 mm in 1 mm pockets (protrude 2 mm below the ribs).",
    )

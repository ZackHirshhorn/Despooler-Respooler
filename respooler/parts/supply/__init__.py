"""WS-D  Supply spool holder.

The source spool rests with its flanges on two free-running rollers, so the hub
bore does not matter.  Everything sits on one printed cradle:

  * supply_cradle        one-piece frame: two end walls (towers carry the roller
                         axles), two floor rails under the rollers, a centre
                         T-slot rail along Y for the side fences, a cross tie and
                         the inclined brake block.  Bolted down on 8 grid points.
  * supply_roller        2x printed sleeve D26 on an M8 rod, 688ZZ bearing in
                         each end (608 does not fit a D26 sleeve with 2.4 walls).
  * supply_fence         2x adjustable side fence riding on the T-slot rail
                         (one M3 thumb screw + T-nut each); keeps the spool on
                         SPOOL_CENTER_Y for any width 40..80.
  * supply_brake_*       spring-loaded felt pad pressing radially on the +Y end
                         of the X=155 roller, adjusted by an M4 screw + knob.

Global frame: rollers parallel to Y at X = 45 / 155, Z = 30.
"""
from math import cos, sin, radians

import cadquery as cq

import params as P
from lib.part import Part, loc, axis_y, export
from lib.common import cyl, insert_hole, FASTENERS

# --------------------------------------------------------------------------
# Key dimensions (global)
# --------------------------------------------------------------------------
CX = P.SUPPLY_CENTER_X
RX = (CX - P.SUPPLY_ROLLER_DX, CX + P.SUPPLY_ROLLER_DX)     # 45, 155
RZ = P.SUPPLY_ROLLER_Z                                      # 30
RD = P.SUPPLY_ROLLER_D                                      # 26
Y0, Y1 = P.SUPPLY_ROLLER_Y                                  # -5, 95 (inner wall faces)
WALL_T = 10.0                                               # end walls Y -15..-5, 95..105
ENV = P.ENVELOPES["supply"]
X_MIN, X_MAX = ENV["x"]                                     # 30, 170

B688 = (8.0, 16.0, 5.0)                                     # 688ZZ bearing
SLEEVE_GAP = 1.0                                            # sleeve end to wall face
SLEEVE_Y = (Y0 + SLEEVE_GAP, Y1 - SLEEVE_GAP)               # -4 .. 94
SLEEVE_L = SLEEVE_Y[1] - SLEEVE_Y[0]
TOWER_R = 15.0

# floor rails under the rollers / centre T-slot rail
RAIL_H = 12.0
RAIL_L = (RX[0] - 13.0, RX[0] + 13.0)                       # 32..58
RAIL_R = (139.0, 169.0)
TRACK_X = (76.0, 110.0)                                     # centre fence rail
TRACK_Y = (2.0, 80.0)
TRACK_H = 10.0
SLOT_X = 88.0                                               # T-slot centre / fence screw axis
M3 = FASTENERS["M3"]
CH_W = M3["nut_af"] + 0.4                                   # nut channel width (5.9)
CH_Z = (3.0, 3.0 + M3["nut_t"] + 0.4)                       # 3.0 .. 5.8
SLOT_W = M3["clear"]                                        # 3.4

FEET = [(45, -5), (45, 35), (45, 95), (145, -5), (145, 35), (145, 95), (105, 15), (105, 75)]

# brake: axis through roller (155, 30) pointing down-left 20 deg; local z = distance
BRAKE_Y = 89.5
BRAKE_LOC = loc(RX[1], BRAKE_Y, RZ, ry=-110)                # local +Z -> (-cos20, 0, -sin20)
BRAKE_BLOCK = dict(half=7.0, s=(15.0, 38.0), y=(82.0, 97.0))
BORE_D = 8.4
FELT = dict(d=8.0, t=3.0, s0=RD / 2 + 0.3)                  # 0.3 gap in model = contact
PLUNGER = dict(d=8.0, l=8.0, pocket_d=5.6, pocket_s=18.0)
INSERT_M4 = FASTENERS["M4"]["insert"]                       # (5.6, 6)
SCREW_TIP_S = 26.0                                          # nominal setting in model
SCREW_L = 20.0
KNOB = dict(d=14.0, t=6.0)

# ghost spool (OD 200, 64 wide, centred on SPOOL_CENTER_Y)
GHOST_OD, GHOST_W = 200.0, 64.0
GHOST_Y = (P.SPOOL_CENTER_Y - GHOST_W / 2, P.SPOOL_CENTER_Y + GHOST_W / 2)
FENCE_GAP = 0.5


def _box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def _wp(shape):
    return cq.Workplane("XY").add(shape)


def _moved(wp, l):
    return _wp(wp.val().moved(l))


def _y_cyl(d, y0, y1, x, z):
    """Cylinder along global Y."""
    return _moved(cyl(d, y1 - y0), axis_y(x, z, y0))


def _hex(af, h, z0=0.0, rot=0.0):
    return (cq.Workplane("XY").workplane(offset=z0).polygon(6, af / cos(radians(30))).extrude(h)
            .rotate((0, 0, 0), (0, 0, 1), rot))


def _flat(shape_wp, rx=0.0, ry=0.0, rz=0.0):
    """print_loc that applies the rotation then drops the part on the bed, centred."""
    s = shape_wp.val().moved(loc(rx=rx, ry=ry, rz=rz))
    bb = s.BoundingBox()
    return loc(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin, rx=rx, ry=ry, rz=rz)


def _brake_pt(s, x=0.0):
    """Global (X, Z) of brake-local point (x, 0, s)."""
    c, sn = cos(radians(20)), sin(radians(20))
    return (RX[1] - s * c - x * sn, RZ - s * sn + x * c)


# --------------------------------------------------------------------------
# Cradle
# --------------------------------------------------------------------------
def cradle():
    yw0 = (Y0 - WALL_T, Y0)
    yw1 = (Y1, Y1 + WALL_T)
    body = None

    def add(s):
        nonlocal body
        body = s if body is None else body.union(s)

    # end walls: low strip + roller towers
    for y0, y1 in (yw0, yw1):
        add(_box(X_MIN, X_MAX, y0, y1, 0, 5.0))
        for x in RX:
            add(_box(x - TOWER_R, x + TOWER_R, y0, y1, 0, RZ))
            add(_y_cyl(2 * TOWER_R, y0, y1, x, RZ))
    # floor rails under rollers (roller bottom Z 17)
    add(_box(*RAIL_L, yw0[0], yw1[1], 0, RAIL_H))
    add(_box(*RAIL_R, yw0[0], yw1[1], 0, RAIL_H))
    # cross tie and centre fence track
    add(_box(RAIL_L[1], RAIL_R[0], P.SPOOL_CENTER_Y - 6, P.SPOOL_CENTER_Y + 6, 0, 8.0))
    add(_box(*TRACK_X, *TRACK_Y, 0, TRACK_H))

    # brake block: block quad + support column down to the base (prints upright)
    h = BRAKE_BLOCK["half"]
    s0, s1 = BRAKE_BLOCK["s"]
    fb, ft = _brake_pt(s0, -h), _brake_pt(s0, h)
    rb, rt = _brake_pt(s1, -h), _brake_pt(s1, h)
    prof = [rt, rb, (rb[0], 0.0), (fb[0], 0.0), fb, ft]
    by0, by1 = BRAKE_BLOCK["y"]
    blk = (cq.Workplane("XZ", origin=(0, by1, 0)).polyline(prof).close().extrude(by1 - by0))
    add(blk)

    # --- cuts ---
    for x in RX:
        body = body.cut(_y_cyl(P.ROD_8 + 0.4, yw0[0] - 1, yw1[1] + 1, x, RZ))
        # -Y: round recess for a spinnable M8 nut; +Y: hex trap (captive nut)
        body = body.cut(_y_cyl(20.0, yw0[0] - 1, yw0[0] + 7.0, x, RZ))
        trap = _hex(13.2, 8.0, rot=30)
        body = body.cut(_moved(trap, axis_y(x, RZ, yw1[1] - 7.0)))
    # T-slot: nut channel + screw slot, open at both track ends
    body = body.cut(_box(SLOT_X - CH_W / 2, SLOT_X + CH_W / 2, TRACK_Y[0] - 1, TRACK_Y[1] + 1, *CH_Z))
    body = body.cut(_box(SLOT_X - SLOT_W / 2, SLOT_X + SLOT_W / 2, TRACK_Y[0] - 1, TRACK_Y[1] + 1,
                         CH_Z[0] + 0.1, TRACK_H + 1))
    # brake bore (plunger) and M4 insert hole
    bore_end = s1 - INSERT_M4[1]
    body = body.cut(_moved(cyl(BORE_D, bore_end - (s0 - 2), s0 - 2), BRAKE_LOC))
    body = body.cut(_moved(cyl(INSERT_M4[0], s1 + 3 - bore_end, bore_end), BRAKE_LOC))
    # heat-set inserts in the feet (screws come up through the base)
    for x, y in FEET:
        assert P.on_grid(x, y)
        body = body.cut(insert_hole("M3").translate((x, y, 0)))
    return body


# --------------------------------------------------------------------------
# Roller sleeve (local axis Z, 0..SLEEVE_L)
# --------------------------------------------------------------------------
def roller_sleeve():
    s = cyl(RD, SLEEVE_L).faces(">Z").edges().chamfer(0.6).faces("<Z").edges().chamfer(0.6)
    s = s.cut(cyl(9.5, SLEEVE_L + 2, -1))
    pd = B688[1] + 2 * P.PRESS_FIT
    s = s.cut(cyl(pd, B688[2] + 1, -1)).cut(cyl(pd, B688[2] + 1, SLEEVE_L - B688[2]))
    return s


# --------------------------------------------------------------------------
# Side fence (local: x centred on screw, y=0 spool-facing face, +y toward spool,
# z=0 on the track top)
# --------------------------------------------------------------------------
FENCE_X = (-14.0, 14.0)
FENCE_H = 45.0
FENCE_T = 4.0
FOOT_L = 16.0
FOOT_T = 3.0
SCREW_Y = 8.0


def fence():
    prof = [(-FENCE_T - 5.0, 0), (FOOT_L, 0), (FOOT_L, FOOT_T), (0, FOOT_T), (0, FENCE_H),
            (-FENCE_T, FENCE_H), (-FENCE_T, 30.0)]
    w = FENCE_X[1] - FENCE_X[0]
    f = cq.Workplane("YZ", origin=(FENCE_X[0], 0, 0)).polyline(prof).close().extrude(w)
    f = f.edges("|X and >Z").fillet(1.0)
    key = _box(-1.5, 1.5, -FENCE_T, FOOT_L, -2.0, 0.0)
    f = f.union(key)
    f = f.cut(cyl(M3["clear"], 20, -5).translate((0, SCREW_Y, 0)))
    return f


# --------------------------------------------------------------------------
# Brake parts (brake-local frame: axis Z = distance from roller axis)
# --------------------------------------------------------------------------
def brake_plunger():
    s0 = FELT["s0"] + FELT["t"]
    p = cyl(PLUNGER["d"], PLUNGER["l"], s0)
    p = p.cut(cyl(PLUNGER["pocket_d"], s0 + PLUNGER["l"] - PLUNGER["pocket_s"] + 1, PLUNGER["pocket_s"]))
    return p


def brake_knob():
    head_s = SCREW_TIP_S + SCREW_L
    k0 = head_s - 1.0
    k = cyl(KNOB["d"], KNOB["t"], k0)
    k = k.edges(">Z").fillet(1.5)
    k = k.cut(_hex(7.3, 5.2, k0 - 1.0, rot=30)).cut(cyl(4.4, 10, k0 - 1))
    # grip flutes
    for i in range(8):
        a = radians(i * 45)
        k = k.cut(cyl(3.0, KNOB["t"] + 2, k0 - 1).translate((KNOB["d"] / 2 * cos(a), KNOB["d"] / 2 * sin(a), 0)))
    return k


# --------------------------------------------------------------------------
# Vitamins
# --------------------------------------------------------------------------
def v_rod():
    return cyl(P.ROD_8 - 0.1, 119.0)


def v_nut_m8():
    return _hex(13.0, 6.5, rot=30).cut(cyl(8.0, 10, -1))


def v_bearing():
    return cyl(B688[1] - 0.2, B688[2]).cut(cyl(B688[0], 10, -1))


def v_shim():
    return cyl(14.0, 0.5).cut(cyl(8.2, 2, -1))


def build():
    parts = []
    cr = cradle()
    parts.append(Part("supply_cradle", cr, loc(), print_loc=_flat(cr),
                      notes="print upright as modelled (feet on bed); 4 perimeters; M3 inserts x8 from below, "
                            "M4 insert at brake block rear; captive M8 nut traps on +Y face"))

    # rollers ------------------------------------------------------------
    sl = roller_sleeve()
    for i, x in enumerate(RX):
        nm = "supply_roller" if i == 0 else f"supply_roller_{'brake' if x == RX[1] else i}"
        parts.append(Part(nm, sl, axis_y(x, RZ, SLEEVE_Y[0]), print_loc=_flat(sl),
                          notes="print standing on end; 688ZZ press fit both ends", color="gray50"))
        # vitamins on this axle
        yw0 = Y0 - WALL_T
        parts.append(Part(f"supply_rod_{i}", v_rod(), axis_y(x, RZ, yw0 + 0.5), printed=False,
                          notes="M8 threaded rod 119 mm (or M8x120 bolt shortened)", color="gray80"))
        parts.append(Part(f"supply_nut_m8_a{i}", v_nut_m8(), axis_y(x, RZ, yw0 + 0.2), printed=False,
                          notes="M8 nut in -Y round recess (tighten last, snug only)", color="gray80"))
        parts.append(Part(f"supply_nut_m8_b{i}", v_nut_m8(), axis_y(x, RZ, Y1 + WALL_T - 6.8), printed=False,
                          notes="M8 nut captive in +Y hex trap", color="gray80"))
        parts.append(Part(f"supply_bearing_a{i}", v_bearing(), axis_y(x, RZ, SLEEVE_Y[0]), printed=False,
                          notes="688ZZ 8x16x5", color="gray80"))
        parts.append(Part(f"supply_bearing_b{i}", v_bearing(), axis_y(x, RZ, SLEEVE_Y[1] - B688[2]),
                          printed=False, notes="688ZZ 8x16x5", color="gray80"))
        parts.append(Part(f"supply_shim_a{i}", v_shim(), axis_y(x, RZ, SLEEVE_Y[0] - 0.75), printed=False,
                          notes="M8 shim washer 8x14x0.5 (DIN 988)", color="gray80"))
        parts.append(Part(f"supply_shim_b{i}", v_shim(), axis_y(x, RZ, SLEEVE_Y[1] + 0.25), printed=False,
                          notes="M8 shim washer 8x14x0.5 (DIN 988)", color="gray80"))

    # fences ----------------------------------------------------------------
    fe = fence()
    f_y = GHOST_Y[0] - FENCE_GAP
    g_y = GHOST_Y[1] + FENCE_GAP
    fence_locs = [("supply_fence", loc(SLOT_X, f_y, TRACK_H)),
                  ("supply_fence_2", loc(SLOT_X, g_y, TRACK_H, rz=180))]
    for i, (nm, l) in enumerate(fence_locs):
        parts.append(Part(nm, fe, l, print_loc=_flat(fe, ry=-90),
                          notes="print on its end face (X up); slide on centre T-slot rail", color="yellow"))
        scr = cyl(2.9, 10.0, -10.0 + FOOT_T + 0.0).union(cyl(5.5, 3.0, FOOT_T)).translate((0, SCREW_Y, 0))
        parts.append(Part(f"supply_fence_screw_{i}", scr, l, printed=False,
                          notes="M3x10 knurled thumb screw (or SHCS)", color="gray80"))
        nut = _hex(M3["nut_af"], M3["nut_t"], rot=30).cut(cyl(3.0, 5, -1)).translate((0, SCREW_Y, 0))
        nut_z = CH_Z[0] + 0.2 - TRACK_H
        parts.append(Part(f"supply_fence_tnut_{i}", nut.translate((0, 0, nut_z)), l, printed=False,
                          notes="M3 hex nut sliding in T-slot channel", color="gray80"))

    # brake -----------------------------------------------------------------
    pl = brake_plunger()
    parts.append(Part("supply_brake_plunger", pl, BRAKE_LOC, print_loc=_flat(pl),
                      notes="print standing, pocket up; felt pad glued to front face", color="red"))
    kn = brake_knob()
    parts.append(Part("supply_brake_knob", kn, BRAKE_LOC, print_loc=_flat(kn),
                      notes="print with hex-recess face on bed; M4 hex bolt head pressed/glued into hex recess", color="red"))
    felt = cyl(FELT["d"], FELT["t"], FELT["s0"])
    parts.append(Part("supply_brake_felt", felt, BRAKE_LOC, printed=False,
                      notes="wool felt pad D8x3 (or 3 mm craft felt), glued to plunger", color="white"))
    sp0 = PLUNGER["pocket_s"] + 0.1
    spring = cyl(5.0, SCREW_TIP_S - 0.1 - sp0, sp0).cut(cyl(4.0, 40, 0))
    parts.append(Part("supply_brake_spring", spring, BRAKE_LOC, printed=False,
                      notes="compression spring OD5 x wire 0.5 x L15 free, ~0.3 N/mm", color="gray80"))
    head_s = SCREW_TIP_S + SCREW_L
    bolt = cyl(3.9, SCREW_L, SCREW_TIP_S).union(_hex(7.0, 2.8, head_s, rot=30))
    parts.append(Part("supply_brake_screw", bolt, BRAKE_LOC, printed=False,
                      notes="M4x20 hex bolt (DIN 933) in M4 heat-set insert", color="gray80"))

    # ghost spool (last) ------------------------------------------------------
    zc = P.supply_spool_center_z(GHOST_OD) + 0.5
    ghost = _y_cyl(GHOST_OD, GHOST_Y[0], GHOST_Y[1], CX, zc)
    parts.append(Part("supply_ghost_spool", ghost, loc(), printed=False,
                      notes="reference 1 kg spool OD200 x 64, raised 0.5 mm off the rollers", color="green"))
    return parts


if __name__ == "__main__":
    export(build(), "supply")

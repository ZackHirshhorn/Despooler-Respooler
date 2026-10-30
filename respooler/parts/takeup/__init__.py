"""WS-B: take-up shaft, bearing tower, spool cones and fixed final gear.

The slip clutch was dropped (motor stalls at the anchored tail; PTC fuse cuts it).
Layout along the take-up axis (X 410, Z 130), Y from back to front:

  Y -90 .. 125   M8 threaded rod (Y -88..-80 left free for the traverse takeoff gear)
  Y -42  .. -35.5 jam nut 3 (in a round counterbore in the gear's -Y face)
  Y -40  .. -32  s4 final gear 60T m1.5, FIXED to the shaft
  Y -32  .. -23  gear hub, hex socket grips lock nut 1 (Y -28.5..-23)
  Y -28.5.. -22  lock nut 1 (jams against 608 inner race)
  Y -22  .. 0    bearing tower: 608 @ -22..-15, inner spacer -15..-7, 608 @ -7..0
  Y   0  .. 6.5  lock nut 2 (captive in the back cone, jams against 608 inner race)
  Y   1  .. 20   back cone: flat drive face at Y = SPOOL_REF_Y (5), pilot to Y 20
  front cone (45 deg, 26..66) + front knob slide on from the +Y end.
"""
from math import cos, sin, radians, atan2, degrees, sqrt

import cadquery as cq

import params as P
from lib.part import Part, loc, axis_y, export
from lib.gears import spur_gear
from lib.common import FASTENERS, insert_hole, cyl

AX, AZ = P.TAKEUP_AXIS
ST = P.STAGES[3]                      # s4: final gear on the take-up shaft
PINION_SHAFT = P.SHAFT_S3
ROD_CLEAR = 8.4            # printed bores on the M8 rod
HEX_AF = FASTENERS["M8"]["nut_af"]
HEX_POCKET_AF = HEX_AF + 0.2
NUT_T = FASTENERS["M8"]["nut_t"]      # 6.5
B_ID, B_OD, B_W = P.BEARING_608
B_OD_MODEL = B_OD + 2 * P.PRESS_FIT   # vitamin modelled at the pocket size (no fake overlap)

# --- Y stations -----------------------------------------------------------
TOWER_Y0, TOWER_Y1 = P.TAKEUP_TOWER_Y          # -22, 0
B1_Y = (TOWER_Y0, TOWER_Y0 + B_W)              # -22 .. -15
B2_Y = (TOWER_Y1 - B_W, TOWER_Y1)              # -7 .. 0
NUT1_Y = (TOWER_Y0 - NUT_T, TOWER_Y0)          # -28.5 .. -22
GEAR_Y = ST["y"]                               # -40 .. -32
HUB_TOP_Y = TOWER_Y0 - 1.0                     # -23 (1 mm gap to tower face)
HUB_D = 26.0
NUT3_Y = (P.FINAL_GEAR_ENVELOPE["y"][0], P.FINAL_GEAR_ENVELOPE["y"][0] + NUT_T)   # -42 .. -35.5
CBORE_D = 21.0                                 # room for a 13 mm socket wrench
BACK_Y0 = TOWER_Y1 + 1.0                       # back cone rear face (1 mm gap)

# ghost spool used to place the front cone (200 x 67, bore 53)
GHOST = dict(od=200.0, w=67.0, bore=53.0, core=90.0)

# --- small helpers ----------------------------------------------------------
def ring(od, id_, h, z0=0.0):
    return cyl(od, h, z0).cut(cyl(id_, h + 2, z0 - 1))


def hex_prism(af, h, z0=0.0):
    return cq.Workplane("XY").workplane(offset=z0).polygon(6, af / cos(radians(30))).extrude(h)


def nut(t=NUT_T, hole=P.ROD_8):
    return hex_prism(HEX_AF, t).cut(cyl(hole, t + 2, -1))


def double_d(d, af, h, z0=0.0):
    c = cyl(d, h, z0)
    return c.intersect(cq.Workplane("XY").workplane(offset=z0).rect(af, d + 2).extrude(h))


def revolve_rz(pts):
    """Revolve an (r, z) profile about local Z."""
    return cq.Workplane("XZ").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0))


def on_axis(y0, rz=0.0):
    return loc(AX, y0, AZ, rx=-90, rz=rz)


def flip(h):
    """print_loc: turn a local-Z part upside down so its z=h face is on the bed."""
    return loc(0, 0, h, rx=180)


# --- printed parts ----------------------------------------------------------
def tower():
    """Modelled directly in global coordinates."""
    r_boss = 17.0
    x_left, x_right = AX - 53.0, 420.0          # 357 .. 420 (base edge)
    # left flank tangent from (x_left, 0) to the boss circle
    dx, dz = AX - x_left, AZ
    d = sqrt(dx * dx + dz * dz)
    ang = atan2(dz, dx) + asin_safe(r_boss / d)
    L = sqrt(d * d - r_boss * r_boss)
    tx, tz = x_left + L * cos(ang), L * sin(ang)
    zr = AZ - sqrt(r_boss ** 2 - (x_right - AX) ** 2)
    prof = (cq.Workplane("XZ").workplane(offset=-TOWER_Y1)   # XZ normal is -Y
            .moveTo(x_left, 0).lineTo(x_right, 0).lineTo(x_right, zr)
            .threePointArc((AX + r_boss, AZ), (AX, AZ + r_boss))
            .threePointArc((AX - r_boss * 0.7071, AZ + r_boss * 0.7071), (tx, tz))
            .close().extrude(TOWER_Y1 - TOWER_Y0))
    # foot plate under the tower and under the spool (low, below spool flanges)
    foot_t = 8.0
    foot_y1 = 40.0
    foot = cq.Workplane("XY").box(x_right - x_left, foot_y1 - TOWER_Y0, foot_t, centered=False) \
        .translate((x_left, TOWER_Y0, 0))
    body = prof.union(foot)
    # low gussets on the +Y face (stay below Z 13: spool flange bottom is >= Z 15)
    for xr in (x_left, 373.0, 393.0, 414.0):
        rib = (cq.Workplane("YZ").workplane(offset=xr)
               .polyline([(TOWER_Y1 - 1, foot_t - 1), (TOWER_Y1 - 1, 13.0), (32.0, foot_t - 1)]).close()
               .extrude(4.0))
        body = body.union(rib)
    # lightening pocket from the +Y face, leaving an 8 mm web at the back and 7 mm rims
    zl0, zl1 = 14.0, 100.0
    slope = (tx - x_left) / tz
    pocket = (cq.Workplane("XZ").workplane(offset=-TOWER_Y1)
              .polyline([(x_left + 7 + slope * zl0, zl0), (x_right - 7, zl0),
                         (x_right - 7, zl1), (x_left + 7 + slope * zl1, zl1)]).close()
              .extrude(14.0))
    body = body.cut(pocket)
    # bearing pockets + shoulder bore (axis along Y)
    pk = B_OD_MODEL
    cut = (cq.Workplane("XZ").workplane(offset=-TOWER_Y0).center(AX, AZ).circle(pk / 2).extrude(-B_W)   # -22..-15
           .union(cq.Workplane("XZ").workplane(offset=-TOWER_Y1).center(AX, AZ).circle(pk / 2).extrude(B_W))  # 0..-7
           .union(cq.Workplane("XZ").workplane(offset=-5).center(AX, AZ).circle(17.0 / 2).extrude(40)))
    body = body.cut(cut)
    # mounting: every grid point under the foot gets a heat-set insert from below
    holes = None
    for gx, gy in P.grid_points():
        if x_left + 5 <= gx <= x_right - 5 and TOWER_Y0 + 5 <= gy <= foot_y1 - 5:
            h = insert_hole("M3").union(cyl(FASTENERS["M3"]["clear"], 2.0, 5.0)).translate((gx, gy, 0))
            holes = h if holes is None else holes.union(h)
    body = body.cut(holes)
    return body


def asin_safe(v):
    from math import asin
    return asin(max(-1.0, min(1.0, v)))


def bearing_spacer():
    return ring(12.6, ROD_CLEAR, B2_Y[0] - B1_Y[1])


def final_gear():
    """s4 gear, local z 0 = Y -40. Disc z 0..8, hub z 8..17 (Y -32..-23).
    Hex socket (top) grips lock nut 1; round counterbore (bottom) holds jam nut 3."""
    face = ST["face"]
    z_top = HUB_TOP_Y - GEAR_Y[0]                    # 17
    g = spur_gear(ST["m"], ST["zg"], face).union(cyl(HUB_D, z_top - face, face))
    for i in range(6):                               # lightening holes in the web
        a = radians(i * 60 + 30)
        g = g.cut(cyl(14.0, face + 2, -1).translate((30.0 * cos(a), 30.0 * sin(a), 0)))
    sock = HUB_TOP_Y - NUT1_Y[0]                     # 5.5
    g = g.cut(hex_prism(HEX_POCKET_AF, sock + 1, z_top - sock))
    g = g.cut(cyl(CBORE_D, NUT3_Y[1] - GEAR_Y[0] + 0.01, -0.01))   # z 0..4.5
    return g.cut(cyl(ROD_CLEAR, 40, -5))


def back_cone():
    """Local z = 0 at Y 1 (rear). Drive face (flange contact) at z = 4 (Y = SPOOL_REF_Y)."""
    zf = P.SPOOL_REF_Y - BACK_Y0                     # 4
    r = 38.0                                         # Ø76 < SPOOL_CORE_MIN 80
    c = revolve_rz([(ROD_CLEAR / 2, 0), (r - 1, 0), (r, 1), (r, zf), (13.0, zf),
                    (13.0, 17.5), (11.5, 19.0), (ROD_CLEAR / 2, 19.0)])
    pocket = (TOWER_Y1 + NUT_T) - BACK_Y0            # nut Y 0..6.5 -> 5.5 deep from z=0
    c = c.cut(hex_prism(HEX_POCKET_AF, pocket, -0.01))
    return c


FC = dict(r_tip=13.0, r_big=33.0, l_cone=20.0, l_cyl=3.0, rec_r=10.5, rec_d=6.0)


def front_cone():
    """45 deg cone, Ø26 tip (z=0) to Ø66 (z=20); knob recess at the rear."""
    f = FC
    zt = f["l_cone"] + f["l_cyl"]
    return revolve_rz([(ROD_CLEAR / 2, 0), (f["r_tip"], 0), (f["r_big"], f["l_cone"]), (f["r_big"], zt),
                       (f["rec_r"], zt), (f["rec_r"], zt - f["rec_d"]), (ROD_CLEAR / 2, zt - f["rec_d"])])


def front_knob():
    """Local z: hub 0..6 (Ø20, enters cone recess), grip 6..14. Nut pocket opens to +Y."""
    k = cyl(20.0, 6.0).union(cyl(50.0, 8.0, 6.0))
    for i in range(10):
        a = radians(i * 36)
        k = k.cut(cyl(8.0, 10, 5).translate((27.5 * cos(a), 27.5 * sin(a), 0)))
    k = k.cut(hex_prism(HEX_POCKET_AF, NUT_T + 0.2, 14.0 - NUT_T - 0.2 + 0.01))
    return k.cut(cyl(ROD_CLEAR, 20, -1))


# --- vitamins -------------------------------------------------------------
def bearing608():
    return ring(B_OD_MODEL, B_ID, B_W)


def ghost_spool():
    g = GHOST
    t = 3.0
    s = ring(g["core"], g["bore"], g["w"])
    s = s.union(ring(g["od"], g["bore"], t)).union(ring(g["od"], g["bore"], t, g["w"] - t))
    return s


# --- build ----------------------------------------------------------------
def build():
    parts = []
    steel = "lightgray"

    # gear phase: put a tooth GAP on the line of centres towards the s4 pinion shaft
    vx, vz = PINION_SHAFT[0] - AX, PINION_SHAFT[1] - AZ
    ang_local = degrees(atan2(-vz, vx))              # axis_y maps local Y to global -Z
    pitch = 360.0 / ST["zg"]
    phase = ang_local - 0.5 * pitch
    try:
        from parts.gearbox import A4G
    except Exception:          # gearbox absent, mid-revision, or not yet exporting A4G
        A4G = None
    if A4G is not None:
        k = (A4G - phase) / pitch
        assert abs(k - round(k)) < 1e-6, f"A4G {A4G} disagrees with take-up phase {phase}"
        phase = A4G

    parts.append(Part("takeup_tower", tower(), loc(), print_loc=loc(-AX, 0, -TOWER_Y0, rx=90),
                      color="orange",
                      notes="Print lying on its -Y face (no supports). 2x 608 press fit, 9x M3 heat-set "
                            "inserts from below on grid points. Shoulder above the lower 608 pocket is a "
                            "2.5 mm overhang ring (bridges fine)."))
    parts.append(Part("takeup_bearing_spacer", bearing_spacer(), on_axis(B1_Y[1]),
                      notes="Inner-race spacer between the 608s, 100% infill. Wall 2.1 mm (< MIN_WALL) "
                            "because it must clear the 608 shields; it is a pure compression ring."))
    parts.append(Part("takeup_final_gear", final_gear(), on_axis(GEAR_Y[0], phase),
                      notes=f"s4 gear {ST['zg']}T m{ST['m']}, fixed to the shaft. Print -Y face down, hub up. "
                            f"Hex socket grips lock nut 1, jam nut 3 in the counterbore clamps it. "
                            f"Phase: tooth gap toward SHAFT_S3 (rz={phase:.3f} deg)."))
    parts.append(Part("takeup_cone_back", back_cone(), on_axis(BACK_Y0),
                      notes="Flat drive face at Y=SPOOL_REF_Y, D76 (< core 80). Print rear face down. "
                            "Captive M8 nut (lock nut 2) in the rear hex pocket."))

    # front cone positioned for the ghost spool (front flange at Y 5+67, bore 53)
    front_flange_y = P.SPOOL_REF_Y + GHOST["w"]
    z_contact = (GHOST["bore"] / 2 - FC["r_tip"]) * (FC["l_cone"] / (FC["r_big"] - FC["r_tip"]))
    fc_y0 = front_flange_y - z_contact
    fc_len = FC["l_cone"] + FC["l_cyl"]
    parts.append(Part("takeup_cone_front", front_cone(), on_axis(fc_y0), print_loc=flip(fc_len),
                      notes="One 45 deg cone covers bores 26..66. Print large end down."))
    fk_y0 = fc_y0 + fc_len - FC["rec_d"]
    parts.append(Part("takeup_knob_front", front_knob(), on_axis(fk_y0), print_loc=flip(14.0),
                      notes="Hand knob, captive M8 nut (pocket opens to +Y). Print grip face down."))

    # vitamins
    parts.append(Part("takeup_rod_m8", cyl(P.ROD_8, P.TAKEUP_SHAFT_Y[1] - P.TAKEUP_SHAFT_Y[0]),
                      on_axis(P.TAKEUP_SHAFT_Y[0]), printed=False, color=steel,
                      notes=f"M8 threaded rod, {P.TAKEUP_SHAFT_Y[1] - P.TAKEUP_SHAFT_Y[0]:.0f} mm (Y {P.TAKEUP_SHAFT_Y[0]:.0f}..{P.TAKEUP_SHAFT_Y[1]:.0f})."))
    for i, (y0, _) in enumerate((B1_Y, B2_Y), 1):
        parts.append(Part(f"takeup_608_{i}", bearing608(), on_axis(y0), printed=False, color=steel))
    parts.append(Part("takeup_nut_lock1", nut(), on_axis(NUT1_Y[0], phase), printed=False, color=steel,
                      notes="M8 nut, jams B1 inner race; gripped by the gear hub hex socket."))
    parts.append(Part("takeup_nut_lock2", nut(), on_axis(TOWER_Y1), printed=False, color=steel,
                      notes="M8 nut, jams B2 inner race; captive in back cone."))
    parts.append(Part("takeup_nut_jam3", nut(), on_axis(NUT3_Y[0]), printed=False, color=steel,
                      notes="M8 nut, clamps the final gear onto lock nut 1 (in gear counterbore)."))
    parts.append(Part("takeup_nut_front", nut(), on_axis(fk_y0 + 14.0 - NUT_T), printed=False, color=steel,
                      notes="M8 nut, captive in front knob."))
    parts.append(Part("takeup_ghost_spool", ghost_spool(), on_axis(P.SPOOL_REF_Y), printed=False,
                      color="blue", notes="Visual only: 200 x 67 spool, bore 53."))
    return parts


if __name__ == "__main__":
    export(build(), "takeup")

"""Electronics box + lid + electrical vitamins (WS-F).

Box shell X 10..100, Y -85..-35, Z 0..39, lid Z 39..42 (inside ELECTRONICS_BOX,
which reaches Y -95: the 10 mm strip Y -95..-85 is left free for the knob and
the DC-jack plug so nothing sticks out past the base edge).

Panel layout:
  -X end wall (operator reaches it from the left end of the machine, never
     blocked by the supply spool):  KCD1 rocker (power) + 12 mm momentary
     "thread" button.
  -Y back wall: PWM speed module, PCB parallel to the wall, pot bushing
     through a Ø7.5 hole, knob outside (set-and-forget per material), and the
     DC barrel jack so the supply cable trails off the back.
  Nothing on the lid: its top stays at Z 42 (< 45) and it lifts off after
  4 x M3 screws without dragging any panel parts.
Fixing: 4 floor bosses with M3 heat-set inserts on grid points, screwed from
under the base like every other subsystem.
"""
import cadquery as cq

import params as P
from lib.common import FASTENERS
from lib.part import Part, loc
from parts.base.plate import WIRE_HOLES, WIRE_HOLE_D

M3 = FASTENERS["M3"]
BX = (10.0, 100.0)
BY = (-85.0, -35.0)
BODY_Z = 39.0
LID_T = 3.0
WALL = 3.0
FLOOR = 3.0
INS_D, INS_DEPTH = M3["insert"]

FLOOR_BOSSES = [(25.0, -65.0), (25.0, -45.0), (65.0, -45.0), (85.0, -45.0)]   # grid points
LID_COLUMNS = [(16.0, -79.0), (94.0, -79.0), (94.0, -41.0), (55.0, -41.0)]
BOSS_D, BOSS_H, COL_D = 9.0, 8.0, 7.0

ROCKER_C = (-64.0, 21.0)       # (Y, Z) on -X wall
BUTTON_C = (-47.0, 21.0)
BUTTON_HOLE = 12.2
KNOB_C = (65.0, 20.5)          # (X, Z) on -Y wall
POT_HOLE = 7.5
JACK_C = (30.0, 22.0)

# Low-voltage PWM board (1803BK-style, 1.8-15 V 2 A, or 3-35 V type): typical
# ~35 x 25 mm PCB, 22 mm deep incl. knob -- smaller than params.PWM_MODULE
# (50x33, the 12 V board), so it fits the old space with margin.
PWM_X = (47.5, 82.5)
PWM_Z = (8.0, 33.0)

# PTC resettable fuse (radial, 60V075-style: ~7.5 x 3 x 12 body) in a floor cradle
PTC_C = (80.0, -58.0)            # (X, Y)
PTC_BODY = (7.5, 3.0, 12.0)


def _box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def _cyl(d, p0, axis, length):
    return cq.Solid.makeCylinder(d / 2, length, cq.Vector(*p0), cq.Vector(*axis))


def box_body():
    x0, x1 = BX
    y0, y1 = BY
    b = _box(x0, x1, y0, y1, 0, BODY_Z)
    b = b.cut(_box(x0 + WALL, x1 - WALL, y0 + WALL, y1 - WALL, FLOOR, BODY_Z + 1))
    # floor bosses (inserts from below) and lid columns (inserts from above)
    b = b.union(cq.Workplane("XY").pushPoints(FLOOR_BOSSES).circle(BOSS_D / 2).extrude(BOSS_H))
    b = b.union(cq.Workplane("XY").pushPoints(LID_COLUMNS).circle(COL_D / 2).extrude(BODY_Z))
    # PWM rear support rail (keeps the PCB from tipping back off the pot bushing)
    b = b.union(_box(PWM_X[0], PWM_X[1], -69.6, -67.0, FLOOR, 14.0))
    # PTC cradle: two walls forming a 3.6 mm slot the radial PTC drops into
    px, py = PTC_C
    for ya, yb in ((py - 4.2, py - 1.8), (py + 1.8, py + 4.2)):
        b = b.union(_box(px - 6.0, px + 6.0, ya, yb, FLOOR, FLOOR + 6.0))
    b = b.cut(cq.Workplane("XY").workplane(offset=-0.1).pushPoints(FLOOR_BOSSES)
              .circle(INS_D / 2).extrude(INS_DEPTH + 1.0))
    b = b.cut(cq.Workplane("XY").workplane(offset=BODY_Z - INS_DEPTH - 1.0).pushPoints(LID_COLUMNS)
              .circle(INS_D / 2).extrude(INS_DEPTH + 2.0))
    # wire pass-through to the base underside
    wx, wy = WIRE_HOLES["ebox"]
    b = b.cut(cq.Workplane("XY").workplane(offset=-1).center(wx, wy).circle(WIRE_HOLE_D / 2).extrude(FLOOR + 2))
    # -X wall: rocker + thread button
    ry, rz = ROCKER_C
    rw, rh = P.ROCKER_CUTOUT          # 19 x 13 -> mounted vertical: 13 along Y, 19 along Z
    b = b.cut(_box(x0 - 1, x0 + WALL + 1, ry - rh / 2, ry + rh / 2, rz - rw / 2, rz + rw / 2))
    b = b.cut(cq.Workplane().add(_cyl(BUTTON_HOLE, (x0 - 1, BUTTON_C[0], BUTTON_C[1]), (1, 0, 0), WALL + 2)))
    # -Y wall: pot bushing + DC jack
    b = b.cut(cq.Workplane().add(_cyl(POT_HOLE, (KNOB_C[0], y0 - 1, KNOB_C[1]), (0, 1, 0), WALL + 2)))
    b = b.cut(cq.Workplane().add(_cyl(P.DC_JACK_D, (JACK_C[0], y0 - 1, JACK_C[1]), (0, 1, 0), WALL + 2)))
    return Part(name="base_ebox", shape=b, color="gray40",
                notes="Print floor down, open top up, no supports (13 mm rocker slot top is a short bridge). "
                      "8x M3 heat-set inserts: 4 in floor bosses (from below), 4 in lid columns (from top). "
                      "If the KCD1 clips do not catch in the 3 mm wall, file the slot edge thinner inside.")


def box_lid():
    x0, x1 = BX
    y0, y1 = BY
    lid = _box(x0, x1, y0, y1, BODY_Z, BODY_Z + LID_T)
    lid = lid.cut(cq.Workplane("XY").workplane(offset=BODY_Z - 1).pushPoints(LID_COLUMNS)
                  .circle(M3["clear"] / 2).extrude(LID_T + 2))
    # vent slots over the PWM module MOSFET
    for i in range(5):
        x = 48.0 + i * 8.0
        lid = lid.cut(_box(x - 1.5, x + 1.5, -78.0, -66.0, BODY_Z - 1, BODY_Z + LID_T + 1))
    return Part(name="base_ebox_lid", shape=lid, color="gray40", print_loc=loc(0, 0, -BODY_Z),
                notes="Print flat. 4x M3x8 into the box column inserts.")


def vitamins():
    x0 = BX[0]
    y0 = BY[0]
    parts = []
    # PWM module: PCB+components block inside, pot bushing + nut + knob outside
    kx, kz = KNOB_C
    pwm = _box(PWM_X[0], PWM_X[1], y0 + WALL + 0.1, -69.8, PWM_Z[0], PWM_Z[1]).val()
    pwm = pwm.fuse(_cyl(7.0, (kx, y0 - 2.0, kz), (0, 1, 0), WALL + 2.1))      # bushing
    pwm = pwm.fuse(_cyl(10.0, (kx, y0 - 1.95, kz), (0, 1, 0), 1.9))           # panel nut
    pwm = pwm.fuse(_cyl(16.0, (kx, y0 - 9.5, kz), (0, 1, 0), 7.5))            # knob
    parts.append(Part(name="base_pwm_module", shape=cq.Workplane().add(pwm), printed=False, color="blue",
                      notes="Low-voltage knob PWM motor speed board (1803BK-style 1.8-15 V 2 A, or 3-35 V type), "
                            "~35x25 mm PCB, pot bushing through the back wall, held by the pot nut + rear rail."))
    # PTC resettable fuse in its cradle
    px, py = PTC_C
    w, t, hgt = PTC_BODY
    ptc = _box(px - w / 2, px + w / 2, py - t / 2, py + t / 2, FLOOR + 0.05, FLOOR + 0.05 + hgt).val()
    parts.append(Part(name="base_ptc_fuse", shape=cq.Workplane().add(ptc), printed=False, color="orange",
                      notes="Radial PTC resettable fuse ~0.75 A hold / 1.5 A trip (e.g. 60V075 / RXEF075), "
                            "in series with the motor; sits in the floor cradle, leads heat-shrunk."))
    # DC jack 5.5x2.1 panel mount
    jx, jz = JACK_C
    jack = _cyl(13.0, (jx, y0 - 1.95, jz), (0, 1, 0), 1.9)
    jack = jack.fuse(_cyl(10.8, (jx, y0 - 0.1, jz), (0, 1, 0), 13.0))
    jack = jack.fuse(_cyl(14.0, (jx, y0 + WALL + 0.05, jz), (0, 1, 0), 2.4))
    parts.append(Part(name="base_dc_jack", shape=cq.Workplane().add(jack), printed=False, color="black",
                      notes="5.5x2.1 mm panel DC jack, Ø11 hole, centre +."))
    # KCD1 rocker
    ry, rz = ROCKER_C
    rw, rh = P.ROCKER_CUTOUT
    rock = _box(x0 - 2.0, x0 - 0.05, ry - 7.5, ry + 7.5, rz - 10.5, rz + 10.5).val()
    rock = rock.fuse(_box(x0 - 0.1, x0 + 22.0, ry - rh / 2 + 0.2, ry + rh / 2 - 0.2,
                          rz - rw / 2 + 0.2, rz + rw / 2 - 0.2).val())
    parts.append(Part(name="base_rocker_switch", shape=cq.Workplane().add(rock), printed=False, color="red",
                      notes="KCD1 SPST rocker, 13x19 snap-in cutout (mounted vertical, I = up)."))
    # 12 mm momentary pushbutton
    by, bz = BUTTON_C
    btn = _cyl(10.0, (x0 - 5.0, by, bz), (1, 0, 0), 3.1)
    btn = btn.fuse(_cyl(14.0, (x0 - 2.0, by, bz), (1, 0, 0), 1.95))
    btn = btn.fuse(_cyl(11.8, (x0 - 0.1, by, bz), (1, 0, 0), 23.0))
    btn = btn.fuse(_cyl(15.0, (x0 + WALL + 0.05, by, bz), (1, 0, 0), 2.0))
    parts.append(Part(name="base_thread_button", shape=cq.Workplane().add(btn), printed=False, color="green",
                      notes="12 mm panel momentary push button (NO), wired in parallel with the auto-stop switch."))
    return parts


def build():
    return [box_body(), box_lid()] + vitamins()

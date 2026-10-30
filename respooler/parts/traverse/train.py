"""Gear train: takeoff on the M8 take-up shaft -> compound idler I1 -> idler I2 -> cam gear."""
import cadquery as cq

from lib.gears import spur_gear
from lib.part import Part, axis_y, loc
from lib.common import cyl, FASTENERS
from . import layout as L


def _rot(wp, deg):
    return wp.rotate((0, 0, 0), (0, 0, 1), deg)


def _flip_print(length):
    """Local part along +Z of given length -> upside down on the bed."""
    return loc(0, 0, length, rx=180)


# gear phases (local rotation about Z) so teeth interleave in the assembly
R_TAKEOFF = L.toward(L.T, L.I1)
R_I1G = L.mesh_rot(R_TAKEOFF, L.Z_TAKEOFF, L.T, L.I1, L.Z_I1G)
R_I1P = L.toward(L.I1, L.I2)
R_I2 = L.mesh_rot(R_I1P, L.Z_I1P, L.I1, L.I2, L.Z_I2)
R_CAM = L.mesh_rot(R_I2, L.Z_I2, L.I2, L.C, L.Z_CAM)


def hex_nut(af, t, bore):
    return (cq.Workplane("XY").polygon(6, af / 0.866025).extrude(t)
            .faces(">Z").workplane().hole(bore))


def build():
    parts = []
    tx, tz = L.T
    a0, a1 = L.LAYER_A
    b0, b1 = L.LAYER_B

    # --- takeoff gear on take-up shaft, clamped by two M8 jam nuts
    tk = _rot(spur_gear(L.M, L.Z_TAKEOFF, a1 - a0, bore=8.3), R_TAKEOFF)
    parts.append(Part("traverse_takeoff_gear", tk, loc=axis_y(tx, tz, a0),
                      print_loc=loc(), color="goldenrod",
                      notes="12T m1.5, 4 mm. Bore 8.3 slides on M8 take-up shaft; clamped between "
                            "two M8 jam nuts (friction drive). Print flat."))
    for nm, (y0, y1) in (("rear", L.NUT_REAR_Y), ("front", L.NUT_FRONT_Y)):
        parts.append(Part(f"traverse_nut_m8_{nm}", hex_nut(13.0, y1 - y0, 8.1),
                          loc=axis_y(tx, tz, y0), printed=False, color="gray",
                          notes="M8 jam nut DIN 439 (4 mm)"))

    # --- compound idler I1: 12T pinion (layer B) + 60T gear (layer A), rides on plate boss
    i1x, i1z = L.I1
    front = L.BOSS_I1_Y[1] - 0.2               # 0.2 axial play under the washer
    ln = front - b0
    pin = _rot(spur_gear(L.M, L.Z_I1P, b1 - b0), R_I1P)
    hub = cyl(14.0, a0 - b1 + 0.2, b1 - b0 - 0.1)
    gear = _rot(spur_gear(L.M, L.Z_I1G, front - a0), R_I1G).translate((0, 0, a0 - b0))
    i1 = pin.union(hub).union(gear).cut(cyl(L.BOSS_D + 0.4, 50, -10))
    parts.append(Part("traverse_idler1", i1, loc=axis_y(i1x, i1z, b0),
                      print_loc=_flip_print(ln), color="goldenrod",
                      notes=f"Compound {L.Z_I1G}T/{L.Z_I1P}T m1.5. Print with the 60T face on the bed. "
                            "Bore 8.4 runs on the printed Ø8 boss; retained by M3x8 + M3 large washer."))
    # retaining washer + screw (vitamins)
    w0 = L.BOSS_I1_Y[1]
    parts.append(Part("traverse_washer_i1", cyl(9.0, 0.8).cut(cyl(3.4, 2, -1)),
                      loc=axis_y(i1x, i1z, w0), printed=False, color="gray",
                      notes="M3 large washer DIN 9021 (Ø9)"))
    scr = cyl(3.0, 5.0, -5.0).union(cyl(5.5, 3.0, 0.8))
    parts.append(Part("traverse_screw_i1", scr, loc=axis_y(i1x, i1z, w0),
                      printed=False, color="gray", notes="M3x8 SHCS into M3 insert in boss"))

    # --- idler I2 (layer B)
    i2x, i2z = L.I2
    i2 = _rot(spur_gear(L.M, L.Z_I2, b1 - 0.2 - b0), R_I2).cut(cyl(L.BOSS_D + 0.4, 50, -10))
    parts.append(Part("traverse_idler2", i2, loc=axis_y(i2x, i2z, b0), print_loc=loc(),
                      color="goldenrod",
                      notes=f"{L.Z_I2}T m1.5 idler. Bore 8.4 on printed Ø8 boss; retained by "
                            "M3x8 + M3 large washer."))
    w2 = L.BOSS_I2_Y[1]
    parts.append(Part("traverse_washer_i2", cyl(9.0, 0.8).cut(cyl(3.4, 2, -1)),
                      loc=axis_y(i2x, i2z, w2), printed=False, color="gray",
                      notes="M3 large washer DIN 9021 (Ø9)"))
    parts.append(Part("traverse_screw_i2", scr, loc=axis_y(i2x, i2z, w2),
                      printed=False, color="gray", notes="M3x8 SHCS into M3 insert in boss"))
    return parts

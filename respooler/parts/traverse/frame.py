"""Frame: train plate (rear, Y -92..-89) and front block (Y -5..0), rods.

Both are modelled in the global frame through axis_y(0, 0, y0): local (x, y, z) = (X, -Z, Y - y0).
"""
import cadquery as cq

import params as P
from lib.part import Part, axis_y, loc
from lib.common import cyl, insert_hole
from . import layout as L


def _poly(pts_xz):
    return [(x, -z) for x, z in pts_xz]


def _box(x0, x1, z0, z1, lz0, lz1):
    """Box from global X/Z ranges and local z (= Y - y0) range."""
    return (cq.Workplane("XY").center((x0 + x1) / 2, -(z0 + z1) / 2)
            .rect(x1 - x0, z1 - z0).extrude(lz1 - lz0).translate((0, 0, lz0)))


def _at(d, h, x, z, lz0):
    return cyl(d, h, lz0).translate((x, -z, 0))


def _inserts(points, y0):
    """M3 heat-set holes going up from global Z=0 at grid points (X, Y)."""
    cut = None
    d, depth = (4.0, 5.0)
    for X, Y in points:
        assert P.on_grid(X, Y), (X, Y)
        c = cq.Workplane("XY").add(cq.Solid.makeCylinder(d / 2, depth, cq.Vector(X, 0, Y - y0),
                                                         cq.Vector(0, -1, 0)))
        cut = c if cut is None else cut.union(c)
    return cut


def train_plate():
    y0, y1 = L.PLATE_Y
    t = y1 - y0
    outline = _poly([(262, 0), (386, 0), (386, 128), (368, 140), (262, 140)])
    pl = cq.Workplane("XY").polyline(outline).close().extrude(t)
    # foot flange to the grid row Y = -85, ribs
    pl = pl.union(_box(262, 386, 0, 6, 0, -79 - y0))
    pl = pl.union(_box(262, 266, 6, 88, t, -80 - y0))
    pl = pl.union(_box(382, 386, 6, 80, t, -80 - y0))
    # lightening hole
    pl = pl.cut(_at(40, 50, 305, 40, -10))
    # idler bosses
    i1x, i1z = L.I1
    i2x, i2z = L.I2
    pl = pl.union(_at(L.BOSS_D, L.BOSS_I1_Y[1] - y1, i1x, i1z, t))
    pl = pl.cut(_at(4.0, 5.0, i1x, i1z, L.BOSS_I1_Y[1] - y0 - 5.0))
    pl = pl.union(_at(L.BOSS_D, L.BOSS_I2_Y[1] - y1, i2x, i2z, t))
    pl = pl.cut(_at(4.0, 5.0, i2x, i2z, L.BOSS_I2_Y[1] - y0 - 5.0))
    # cam rod hole + thrust boss for the 625 inner ring
    cx, cz = L.C
    pl = pl.union(_at(7.0, 0.4, cx, cz, t))
    pl = pl.cut(_at(5.1, 20, cx, cz, -5))
    for rx in L.ROD_XS:
        pl = pl.cut(_at(6.1, 20, rx, L.ROD_Z, -5))
    pl = pl.cut(_inserts([(285, -85), (325, -85), (365, -85)], y0))
    return Part("traverse_train_plate", pl, loc=axis_y(0, 0, y0), print_loc=loc(),
                color="orange",
                notes="Rear plate of the traverse: idler bosses (I1 Ø8 + M3 insert, I2 Ø8), cam rod "
                      "and 6 mm rod sockets, foot on grid (285,-85),(325,-85),(365,-85) with M3 "
                      "inserts. Print lying on its rear face; bosses/ribs up.")


def front_block():
    y0, y1 = L.FRONT_Y
    t = y1 - y0
    x0, x1 = L.FRONT_X
    fb = _box(x0, x1, 0, L.ROD_Z + 6.0, 0, t)
    fb = fb.cut(_box(*L.WINDOW_X, *L.WINDOW_Z, -1, t + 1))     # arm window
    fb = fb.cut(_box(277, 291, 15, 95, -1, t + 1))             # lightening
    fb = fb.union(_box(275, 295, 0, 6, -28 - y0, t))           # foot
    fb = fb.union(_box(283, 287, 6, 30, -28 - y0, 0))          # foot rib
    cx, cz = L.C
    fb = fb.union(_at(7.0, 0.4, cx, cz, -0.4))
    fb = fb.cut(_at(5.1, 20, cx, cz, -5))
    for rx in L.ROD_XS:
        fb = fb.cut(_at(6.1, 20, rx, L.ROD_Z, -5))
    fb = fb.cut(_inserts([(285, -5), (285, -25)], y0))
    return Part("traverse_front_block", fb, loc=axis_y(0, 0, y0), print_loc=axis_y(0, 0, 0),
                color="orange",
                notes="Front support: 6 mm rods, Ø5 cam rod, window for the fairlead arm. Foot on "
                      "grid (285,-5),(285,-25) with M3 inserts. Print standing upright (as "
                      "installed); the 11 mm arm window is a bridge.")


def build():
    parts = [train_plate(), front_block()]
    y0, y1 = L.ROD_Y
    for i, rx in enumerate(L.ROD_XS):
        parts.append(Part(f"traverse_rod_{i}", cyl(6.0, y1 - y0), loc=axis_y(rx, L.ROD_Z, y0),
                          printed=False, color="lightgray",
                          notes=f"Ø6 smooth rod (h6 / drill rod) {y1 - y0:.0f} mm"))
    return parts

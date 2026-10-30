"""Shared printable features. All functions return CadQuery Workplanes to CUT
(pockets/holes) or UNION (bosses), placed at the local origin, axis = +Z."""
import cadquery as cq

import params as P

# Metric fastener data: clearance hole, nut across-flats, nut thickness,
# heat-set insert hole (d, depth), head d
FASTENERS = {
    "M2": dict(clear=2.4, nut_af=4.0, nut_t=1.6, insert=(3.2, 4.0), head=3.8),
    "M3": dict(clear=3.4, nut_af=5.5, nut_t=2.4, insert=(4.0, 5.0), head=5.6),
    "M4": dict(clear=4.5, nut_af=7.0, nut_t=3.2, insert=(5.6, 6.0), head=7.2),
    "M5": dict(clear=5.5, nut_af=8.0, nut_t=4.0, insert=(6.4, 7.0), head=8.6),
    "M8": dict(clear=8.5, nut_af=13.0, nut_t=6.5, insert=None, head=13.0),
}


def cyl(d, h, z0=0.0):
    return cq.Workplane("XY").workplane(offset=z0).circle(d / 2).extrude(h)


def bearing_pocket(bearing, extra_depth=0.0, through_d=None):
    """Pocket for a ball bearing (bore, OD, width) with optional through hole."""
    _, od, w = bearing
    cut = cyl(od + 2 * P.PRESS_FIT, w + extra_depth)
    if through_d:
        cut = cut.union(cyl(through_d, 200, -100))
    return cut


def clearance_hole(size, depth, z0=0.0):
    return cyl(FASTENERS[size]["clear"], depth, z0)


def insert_hole(size, z0=0.0):
    d, depth = FASTENERS[size]["insert"]
    return cyl(d, depth, z0)


def nut_trap(size, z0=0.0, extra=0.2):
    f = FASTENERS[size]
    return (cq.Workplane("XY").workplane(offset=z0)
            .polygon(6, (f["nut_af"] + extra) / 0.866025).extrude(f["nut_t"] + extra))


def counterbore(size, depth, cb_depth, z_top=0.0):
    """Screw clearance + head counterbore; head at z_top going -Z."""
    f = FASTENERS[size]
    return cyl(f["clear"], depth, z_top - depth).union(cyl(f["head"] + 0.6, cb_depth, z_top - cb_depth))


def d_shaft_bore(d, flat_depth=0.5, length=20.0, clearance=0.1):
    """Bore with a flat (for D-cut rods or printed D shafts)."""
    r = d / 2 + clearance
    c = cyl(2 * r, length)
    keep = cq.Workplane("XY").center(-flat_depth / 2, 0).rect(2 * r - flat_depth, 2 * r + 1).extrude(length)
    return c.intersect(keep)


def mount_pattern(points, size="M3", depth=10.0, z_top=0.0):
    """Clearance holes for bolting a part to the base (points in local XY)."""
    cut = None
    for x, y in points:
        h = clearance_hole(size, depth, z_top - depth).translate((x, y, 0))
        cut = h if cut is None else cut.union(h)
    return cut

"""Involute spur gear generator (CadQuery). Gear axis = local Z, face from z=0 to z=face."""
from math import pi, cos, sin, tan, acos

import cadquery as cq

import params as P


def _inv(a):
    return tan(a) - a


def gear_profile(m, z, pa_deg=P.PRESSURE_ANGLE, backlash=P.GEAR_BACKLASH / 2,
                 tip_relief=0.05, n_flank=12, n_root=4):
    """Return closed 2D point list (CCW) for an external involute spur gear."""
    a = pa_deg * pi / 180
    rp = m * z / 2
    rb = rp * cos(a)
    ra = rp + m * (1 - tip_relief)
    rf = rp - 1.25 * m
    psi_p = (pi * m / 2 - backlash) / (2 * rp)  # half tooth angle at pitch circle

    def psi(r):
        return psi_p + _inv(a) - _inv(acos(min(1.0, rb / r)))

    r0 = max(rb, rf)
    radii = [r0 + (ra - r0) * i / n_flank for i in range(n_flank + 1)]
    pitch = 2 * pi / z
    pts = []
    for k in range(z):
        c = k * pitch
        flank = []
        if rf < rb:
            flank.append((rf, psi(rb)))
        flank += [(r, psi(r)) for r in radii]
        for r, h in flank:                       # right flank, going out
            pts.append((r * cos(c - h), r * sin(c - h)))
        for r, h in reversed(flank):             # left flank, going in
            pts.append((r * cos(c + h), r * sin(c + h)))
        # root arc to the next tooth
        h0 = psi(rb) if rf < rb else psi(r0)
        a0, a1 = c + h0, c + pitch - h0
        for i in range(1, n_root):
            t = a0 + (a1 - a0) * i / n_root
            pts.append((rf * cos(t), rf * sin(t)))
    return pts


def spur_gear(m, z, face, bore=None, hub_d=None, hub_len=0.0, **kw):
    """Spur gear solid. Optional round bore and a hub on the +Z side."""
    g = cq.Workplane("XY").polyline(gear_profile(m, z, **kw)).close().extrude(face)
    if hub_d and hub_len:
        g = g.union(cq.Workplane("XY").workplane(offset=face).circle(hub_d / 2).extrude(hub_len))
    if bore:
        g = g.faces("<Z").workplane().hole(bore)
    return g


def pitch_d(m, z):
    return m * z


def outside_d(m, z):
    return m * (z + 2)


def center_distance(m, z1, z2):
    return m * (z1 + z2) / 2

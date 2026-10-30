"""Swappable diamond (crossing) drum cams + matching swivelling boat followers.

Groove centreline: y(t) = Yc + S/2 * asin(k cos(t / 2N)) / asin(k), t in [0, 4 pi N):
two helices (N revs out, N revs back) joined by rounded reversals. The groove is cut as the
swept envelope of the boat itself (boat-shaped tools placed along the path, each turned to the
local groove direction), so the boat is guaranteed to fit through crossings and reversals.
"""
from math import cos, sin, asin, atan, atan2, degrees, radians, pi, sqrt

import cadquery as cq

import params as P
from lib.gears import spur_gear
from lib.part import Part, axis_y, loc
from lib.common import cyl, bearing_pocket
from . import layout as L
from .train import R_CAM

SAMPLES_PER_REV = 24
CAM_LEN = L.DRUM_Y[1] - L.CAM_Y0          # 83
GEAR_FACE = L.LAYER_B[1] - L.LAYER_B[0]   # 4
HUB_END = L.HUB_Y[1] - L.CAM_Y0           # 8.5
YC = L.GROOVE_CENTER_Y - L.CAM_Y0         # local z of groove centre
TOP_T = pi * L.N_REVS                     # path parameter at the top of the drum in the assembly


def y_of(t, S):
    k = L.GROOVE_K
    return YC + S / 2 * asin(k * cos(t / (2 * L.N_REVS))) / asin(k)


def dy_of(t, S):
    k = L.GROOVE_K
    u = t / (2 * L.N_REVS)
    return -S / 2 / asin(k) * k * sin(u) / (2 * L.N_REVS) / sqrt(1 - (k * cos(u)) ** 2)


def radii(S):
    r_o = L.DRUM_OD[int(S)] / 2
    r_root = r_o - L.GROOVE_DEPTH
    a_root = atan(L.lead_rate(S) / r_root)
    rho = r_root / cos(a_root) ** 2      # floor curvature along the helix
    return r_o, r_root, rho


def _keel(S, length, width, lift, top):
    """Boat-keel solid in the canonical frame: radial = +X, groove direction = +Y,
    width = Z. Bottom is an arc of radius rho+lift (floor + lift)."""
    r_o, r_root, rho = radii(S)
    R = rho + lift
    drop = R - sqrt(max(R * R - (length / 2) ** 2, 0.0))
    x0 = r_root + lift - drop - 0.5
    box = cq.Solid.makeBox(top - x0, length, width, cq.Vector(x0, -length / 2, -width / 2))
    disk = cq.Solid.makeCylinder(R, width + 2, cq.Vector(r_root - rho, 0, -width / 2 - 1),
                                 cq.Vector(0, 0, 1))
    return box.cut(disk)


def cam_shape(S):
    r_o, r_root, rho = radii(S)
    g = spur_gear(L.M, L.Z_CAM, GEAR_FACE).rotate((0, 0, 0), (0, 0, 1), R_CAM)
    body = g.union(cyl(L.HUB_D, HUB_END - GEAR_FACE + 0.1, GEAR_FACE - 0.1))
    drum = cyl(2 * r_o, CAM_LEN - HUB_END + 0.1, HUB_END - 0.1)
    tool0 = _keel(S, L.BOAT_L + 0.8, L.GROOVE_W, 0.0, r_o + 1.0)
    tools = []
    n = SAMPLES_PER_REV * 2 * L.N_REVS
    rot0 = -90.0 - degrees(TOP_T)            # puts t = TOP_T at the top (global +Z)
    rp = r_o - L.GROOVE_DEPTH / 2
    ts = [4 * pi * L.N_REVS * i / n for i in range(n + 1)]
    for _ in range(3):                       # refine where the boat turns fast (reversals)
        out = [ts[0]]
        for a, b in zip(ts, ts[1:]):
            if abs(atan2(dy_of(a, S), rp) - atan2(dy_of(b, S), rp)) > radians(6):
                out.append((a + b) / 2)
            out.append(b)
        ts = out
    for t in ts[:-1]:
        beta = degrees(atan2(dy_of(t, S), r_o - L.GROOVE_DEPTH / 2))
        lc = (cq.Location(cq.Vector(0, 0, y_of(t, S)))
              * cq.Location(cq.Vector(), cq.Vector(0, 0, 1), degrees(t) + rot0)
              * cq.Location(cq.Vector(), cq.Vector(1, 0, 0), beta))
        tools.append(tool0.moved(lc))
    sweep = tools[0].fuse(*tools[1:]).clean()      # fuse first: much faster/robuster cut
    drum_s = drum.val().cut(sweep)
    body = body.union(cq.Workplane("XY").add(drum_s))
    body = body.cut(cyl(5.2, CAM_LEN + 2, -1))
    body = body.cut(bearing_pocket(P.BEARING_625))
    return cq.Workplane("XY").add(body.val().clean())


def boat_shape(S):
    """Boat in its own frame: z = height above the cam axis, x = groove direction."""
    r_o, r_root, rho = radii(S)
    z_top = L.CARR_Z[0] - 0.3 - L.C[1]
    keel = _keel(S, L.BOAT_L, L.BOAT_W, 0.3, r_o + 0.7)
    # canonical frame (radial X, along Y, width Z) -> boat frame (radial Z, along X, width Y)
    keel = keel.moved(cq.Location(cq.Vector(), cq.Vector(1, 1, 1), -120))
    b = cq.Workplane("XY").add(keel)
    bz0 = r_o + 0.6
    body = (cq.Workplane("XY").box(L.BOAT_L, L.BOAT_BODY_W, z_top - bz0, centered=(True, True, False))
            .translate((0, 0, bz0)))
    ch = min(1.5, (z_top - bz0) - 0.6)
    body = body.edges("<Z and |X").chamfer(ch)
    b = b.union(body)
    post_top = L.CARR_Z[1] + 0.3 - L.C[1]
    b = b.union(cyl(L.POST_D, post_top - z_top + 0.1, z_top - 0.1))
    b = b.cut(cyl(2.5, 10.0, post_top - 10.0))
    return b


def cam_part(S):
    s = int(S)
    return Part(f"traverse_cam_{s}", cam_shape(S),
                loc=axis_y(L.C[0], L.C[1], L.CAM_Y0), print_loc=loc(), color="skyblue",
                notes=(f"Diamond drum cam, stroke {s} mm, N={L.N_REVS} revs per stroke, drum "
                       f"Ø{L.DRUM_OD[s]}, groove {L.GROOVE_W} x {L.GROOVE_DEPTH} deep, max pressure "
                       f"angle {L.pressure_angle(S):.1f} deg. {L.Z_CAM}T m1.5 gear + Ø22 hub with a "
                       f"625ZZ; plain Ø5.2 bore at the front. Use with traverse_boat_{s}. "
                       f"Print gear-face down."))


def boat_part(S, yf=L.FOLLOW_Y_NOM):
    s = int(S)
    beta = degrees(atan2(dy_of(TOP_T, S), L.r_pitch(S)))
    shp = boat_shape(S)
    return Part(f"traverse_boat_{s}", shp,
                loc=loc(L.FX, yf, L.C[1], rz=beta),
                print_loc=loc(0, 0, -shp.val().BoundingBox().zmin), color="red",
                notes=(f"Swivelling boat follower for cam_{s}: keel {L.BOAT_L} x {L.BOAT_W}, "
                       f"Ø6 pivot post in the carriage, M3x10 + M3 washer on top (self-taps "
                       f"into the post). Print upright on the two keel tips; the arched keel underside bridges ~11 mm."))


def build(stroke=56.0):
    parts = [cam_part(stroke), boat_part(stroke)]
    bw = P.BEARING_625[2]
    ring = cyl(15.8, bw).cut(cyl(5.2, bw + 2, -1))
    parts.append(Part("traverse_bearing_625", ring, loc=axis_y(L.C[0], L.C[1], L.CAM_Y0),
                      printed=False, color="lightgray", notes="625ZZ 5x16x5 in the cam gear hub"))
    y0, y1 = L.CAM_ROD_Y
    parts.append(Part("traverse_cam_rod", cyl(5.0, y1 - y0), loc=axis_y(L.C[0], L.C[1], y0),
                      printed=False, color="lightgray",
                      notes=f"Ø5 steel rod {y1 - y0:.0f} mm, stationary (cam spins on it)"))
    return parts

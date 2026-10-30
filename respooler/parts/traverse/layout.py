"""Traverse layout: every coordinate the traverse parts share (global frame).

Train (all m1.5, layers inside TRAVERSE_TRAIN_Y = -92..-76):
  plate      Y -92.0 .. -89.0   train plate: idler bosses, cam rod, 6 mm rods
  layer B    Y -88.5 .. -84.5   I1 pinion 13T, I2 idler 46T, cam gear 21T
  layer A    Y -84.0 .. -80.0   takeoff 12T (on M8 take-up shaft), I1 gear 36T
  R = 36/12 * 21/13 = 4.846 spool turns per cam rev.
  Diamond (crossing) groove, N = 2 cam revs per one-way stroke
  -> 9.69 spool turns per stroke, 19.38 per full cycle (non-integer: no ribboning).
"""
from math import cos, sin, radians, hypot, atan2, atan, degrees, sqrt, asin

import params as P

M = 1.5
Z_TAKEOFF, Z_I1G, Z_I1P, Z_I2, Z_CAM = 12, 36, 13, 46, 21
RATIO = (Z_I1G / Z_TAKEOFF) * (Z_CAM / Z_I1P)      # spool turns per cam rev

PLATE_Y = (-92.0, -89.0)
LAYER_B = (-88.5, -84.5)
LAYER_A = (-84.0, -80.0)

T = P.TAKEUP_AXIS
C = P.CAM_AXIS


def cd(z1, z2):
    return M * (z1 + z2) / 2


_a = radians(200.0)
I1 = (T[0] + cd(Z_TAKEOFF, Z_I1G) * cos(_a), T[1] + cd(Z_TAKEOFF, Z_I1G) * sin(_a))


def _circle_int(p0, r0, p1, r1, lower=True):
    dx, dz = p1[0] - p0[0], p1[1] - p0[1]
    d = hypot(dx, dz)
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = sqrt(r0 * r0 - a * a)
    px, pz = p0[0] + a * dx / d, p0[1] + a * dz / d
    sols = [(px + h * dz / d, pz - h * dx / d), (px - h * dz / d, pz + h * dx / d)]
    return min(sols, key=lambda s: s[1]) if lower else max(sols, key=lambda s: s[1])


I2 = _circle_int(I1, cd(Z_I1P, Z_I2), C, cd(Z_I2, Z_CAM))

# M8 jam nuts (DIN 439, 4 mm) clamp the takeoff gear on the take-up shaft
NUT_REAR_Y = (-88.0, -84.0)
NUT_FRONT_Y = (-80.0, -76.0)

# Idler bosses (printed on the plate): OD 8, gear bores 8.4, M3 insert + screw + Ø9 washer
BOSS_D = 8.0
BOSS_I1_Y = (PLATE_Y[1], -80.0)
BOSS_I2_Y = (PLATE_Y[1], LAYER_B[1])

# ---------------------------------------------------------------- cam
N_REVS = 2                             # cam revs per one-way stroke
TURNS_PER_STROKE = N_REVS * RATIO
CAM_Y0 = LAYER_B[0]                    # rear face of cam gear
HUB_Y = (LAYER_B[1], -80.0)            # Ø22 hub (holds the 625)
HUB_D = 22.0
DRUM_Y = (HUB_Y[1], -5.5)
GROOVE_DEPTH = 3.0
GROOVE_W = 3.6                         # boat keel 3.2
GROOVE_CENTER_Y = -45.0
GROOVE_K = 0.98                        # smoothed triangle asin(k cos)/asin(k): rounded reversals
DRUM_OD = {36: 16.5, 46: 17.0, 56: 18.0, 64: 18.0}
CAM_ROD_Y = (PLATE_Y[0], 0.0)          # Ø5 steel rod, stationary

# boat follower
BOAT_L = 11.0
BOAT_W = 3.2
BOAT_BODY_W = 6.5
POST_D = 6.0


def lead_rate(stroke):
    """max dy/dtheta (mm/rad) of the groove centreline."""
    k = GROOVE_K
    return stroke / 2 * k / asin(k) / (2 * N_REVS)


def r_pitch(stroke):
    return DRUM_OD[int(stroke)] / 2 - GROOVE_DEPTH / 2


def pressure_angle(stroke):
    return degrees(atan(lead_rate(stroke) / r_pitch(stroke)))


# ---------------------------------------------------------------- frame / slider
FRONT_Y = (-5.0, 0.0)
FRONT_X = (266.5, 299.5)

ROD_XS = (272.0, 294.0)
ROD_Z = 131.5
ROD_Y = (PLATE_Y[0], 0.0)

FX = 285.0                             # follower pivot X (above cam axis)
CARR_X = (266.5, 299.5)
CARR_Z = (124.0, 137.5)
CARR_DY = (-7.0, 7.0)                  # rear face is the print bed face (no supports)
ARM_X = (278.0, 288.0)
ARM_Z = (124.0, 130.0)
ARM_DY = (-7.0, 113.0)
SLOT_X = 283.0
SLOT_DY = (81.0, 110.0)
WINDOW_X = (277.5, 288.5)
WINDOW_Z = (123.5, 130.5)

# fairlead (relative to eyelet point Ye)
FL_CLAMP_DY = 13.2
FL_Y = (-8.6, 17.0)
VR_X = 274.0                           # vertical-axis rollers (upstream)
HR_X = 283.0                           # horizontal-axis rollers (downstream)
ROLLER_D = 7.0
ROLLER_L = 8.0
GAP = 3.2                              # filament opening (>= 3, passes 2.85)
VR_DY = GAP / 2 + ROLLER_D / 2         # 5.1
HR_DZ = GAP / 2 + ROLLER_D / 2

FOLLOW_Y_NOM = GROOVE_CENTER_Y
EYE_Y_NOM = P.SPOOL_CENTER_Y
OFFSET_RANGE = (72.0, 95.0)            # Ye - Yf  -> stroke centre Y 27 .. 50
EYE_Y_MIN = FRONT_Y[1] - FL_Y[0] + 0.5  # fairlead must stay in front of the front block


def mesh_rot(rA, zA, cA, cB, zB):
    """Rotation (deg, local Z) for gear B so it meshes with gear A (rotation rA).
    Centres in global (X, Z); gears placed with axis_y -> local (x, y) = (X, -Z)."""
    ax, ay = cA[0], -cA[1]
    bx, by = cB[0], -cB[1]
    th = degrees(atan2(by - ay, bx - ax))
    pA, pB = 360.0 / zA, 360.0 / zB
    t = (rA - th) % pA
    if t > pA / 2:
        t -= pA
    tB = -t * zA / zB + pB / 2
    return th + 180.0 + tB


def toward(c_from, c_to):
    return degrees(atan2(-(c_to[1] - c_from[1]), c_to[0] - c_from[0]))

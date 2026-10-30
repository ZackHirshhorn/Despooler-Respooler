"""Single source of truth for the respooler.

Owned by the integrator. Workstream agents READ this file; they never edit it.
If a value here is wrong for your part, report it in your final message.

Global frame (see interfaces.md):
  X  along the base, filament flows +X (supply -> take-up)
  Y  along the spool axes; +Y = toward the operator / free end of take-up shaft
  Z  up; Z = 0 is the TOP surface of the base plate
All gear and spool axes are parallel to Y. Units: mm, degrees.
"""
from math import cos, sin, radians, hypot

# --------------------------------------------------------------------------
# Printer / fabrication
# --------------------------------------------------------------------------
BED_MAX = (210.0, 210.0, 200.0)   # usable print volume per part (220 bed - margin)
FIT_CLEARANCE = 0.2               # radial clearance for sliding fits
PRESS_FIT = -0.05                 # radial interference for bearing pockets (tune on print)
MIN_WALL = 2.4                    # minimum printed wall
NOZZLE = 0.4

# --------------------------------------------------------------------------
# Spool envelope (researched: no standard exists; covers common 1 kg spools)
#   sources: Sigma Filament sizing guide, Flashforge / American Filament specs
#   typical: OD 197-205, width 55-75 (commonly 64-68), bore 50-58 (52-54 most
#   common, outliers ~32), core 85-105
# --------------------------------------------------------------------------
SPOOL_OD_MIN, SPOOL_OD_MAX = 180.0, 230.0
SPOOL_W_MIN, SPOOL_W_MAX = 40.0, 80.0          # overall width incl. flanges
SPOOL_BORE_MIN, SPOOL_BORE_MAX = 28.0, 62.0    # take-up cones must cover this
SPOOL_CORE_MIN, SPOOL_CORE_MAX = 80.0, 105.0
SPOOL_MASS_MAX = 1.4                           # kg, full spool incl. reel
SPOOL_CENTER_Y = 45.0                          # nominal mid-plane of spools

# --------------------------------------------------------------------------
# Filament
# --------------------------------------------------------------------------
FILAMENT_D = 1.75
FILAMENT_D_MAX = 2.85                          # guides/eyelets must pass this
WIND_TENSION_N = (2.0, 4.0)                    # design range

# --------------------------------------------------------------------------
# Base plate (two printed halves, split at X = BASE_SPLIT_X)
# --------------------------------------------------------------------------
BASE_X = (0.0, 420.0)
BASE_Y = (-95.0, 110.0)
BASE_SPLIT_X = 210.0
BASE_THICK = 6.0                               # plate thickness below Z = 0
BASE_RIB_H = 12.0                              # ribs hang below the plate (Z -18 .. -6)
# Mounting grid: the base has an M3 through-hole (counterbored from below) at every
# grid point. Every subsystem foot puts its screw holes ON grid points and carries
# an M3 heat-set insert (or nut trap) there; screws go up from under the base.
MOUNT_SCREW = "M3"
BASE_GRID_PITCH = 20.0
BASE_GRID_ORIGIN = (5.0, -85.0)     # (X, Y); no grid point lands on the split line

def grid_points():
    pts = []
    x = BASE_GRID_ORIGIN[0]
    while x < BASE_X[1]:
        y = BASE_GRID_ORIGIN[1]
        while y < BASE_Y[1]:
            pts.append((x, y))
            y += BASE_GRID_PITCH
        x += BASE_GRID_PITCH
    return pts

def on_grid(x, y, tol=0.01):
    gx = (x - BASE_GRID_ORIGIN[0]) / BASE_GRID_PITCH
    gy = (y - BASE_GRID_ORIGIN[1]) / BASE_GRID_PITCH
    return abs(gx - round(gx)) * BASE_GRID_PITCH < tol and abs(gy - round(gy)) * BASE_GRID_PITCH < tol

# --------------------------------------------------------------------------
# Vitamins (bought parts). (bore, OD, width)
# --------------------------------------------------------------------------
BEARING_608 = (8.0, 22.0, 7.0)
BEARING_625 = (5.0, 16.0, 5.0)
BEARING_623 = (3.0, 10.0, 4.0)
BEARING_MR105 = (5.0, 10.0, 4.0)
ROD_8 = 8.0          # M8 threaded rod (take-up shaft)
ROD_5 = 5.0          # steel rod
ROD_3 = 3.0          # steel rod
ROD_6 = 6.0          # smooth rod for traverse slider
MICROSWITCH = dict(body=(20.0, 6.4, 10.2), hole_spacing=9.5, hole_d=2.3,
                   lever_len=16.0)            # KW11-3Z / SS-5GL style

# --------------------------------------------------------------------------
# Motor: 130-size can (geometry only, from vendor/motor_130.step).
# Model frame: axis = +Z, shaft points +Z. NO mounting holes -> embracing mount.
# --------------------------------------------------------------------------
MOTOR_STEP = "vendor/motor_130.step"
MOTOR = dict(
    can_d=20.1,          # round sides
    can_flats=15.1,      # across flats (model Y extent of can)
    body_z=(-3.3, 25.0), # rear bushing/endcap .. front face
    boss_d=6.15, boss_z=(25.0, 26.7),
    shaft_d=2.0, shaft_z=(26.7, 36.1),
    terminals="rear endcap, +Y side of flats in model frame (z~1..5)",
)

# --------------------------------------------------------------------------
# Drive train. All axes parallel to Y. Positions are (X, Z).
# 4-stage compound spur, 300:1. Motor runs at <= 6 V; spool ~25-35 rpm
# (~30 min per kg). No slip clutch: the final gear is fixed to the take-up
# shaft; at the anchored spool tail the motor stalls and a PTC fuse trips.
# --------------------------------------------------------------------------
MOTOR_VOLTAGE_MAX = 6.0
PRESSURE_ANGLE = 20.0
GEAR_BACKLASH = 0.15   # total, split between mating gears

STAGES = [
    # name, module, z_pinion, z_gear, face width, Y range of layer
    dict(name="s1", m=0.8,  zp=12, zg=48, face=4.0, y=(-65.0, -61.0)),
    dict(name="s2", m=1.0,  zp=12, zg=45, face=5.0, y=(-57.0, -52.0)),
    dict(name="s3", m=1.25, zp=12, zg=48, face=6.0, y=(-49.0, -43.0)),
    dict(name="s4", m=1.5,  zp=12, zg=60, face=8.0, y=(-40.0, -32.0)),  # gear = final gear, fixed on take-up shaft (WS-B)
]

def center_distance(st):
    return st["m"] * (st["zp"] + st["zg"]) / 2.0

def _step(p, dist, ang_deg):
    return (p[0] + dist * cos(radians(ang_deg)), p[1] + dist * sin(radians(ang_deg)))

TAKEUP_AXIS = (410.0, 130.0)                                      # (X, Z)
SHAFT_S3 = _step(TAKEUP_AXIS, center_distance(STAGES[3]), 225.0)  # carries s3 gear + s4 pinion
SHAFT_S2 = _step(SHAFT_S3, center_distance(STAGES[2]), 250.0)     # carries s2 gear + s3 pinion
SHAFT_S1 = _step(SHAFT_S2, center_distance(STAGES[1]), 180.0)     # carries s1 gear + s2 pinion
MOTOR_AXIS = _step(SHAFT_S1, center_distance(STAGES[0]), 270.0)   # carries s1 pinion

GEAR_RATIO = 1.0
for _s in STAGES:
    GEAR_RATIO *= _s["zg"] / _s["zp"]

# Motor orientation in the assembly: shaft points -Y.
MOTOR_FRONT_FACE_Y = -57.0          # front face of can (model z=25) sits here; pinion on shaft in s1 layer
# Shaft spans: WS-A owns these and may adjust them (report final values).
SHAFT_S1_Y = (-70.0, -38.0)         # 3 mm D-shaft, 623 bearings
SHAFT_S2_Y = (-71.0, -37.0)         # 5 mm D-shaft, 625 bearings
SHAFT_S3_Y = (-55.0, -27.0)         # 5 mm D-shaft, 625 rear + MR105 front
GEARBOX_Y = (-74.0, -24.0)          # housing envelope in Y (owned by WS-A)

# --------------------------------------------------------------------------
# Take-up (WS-B). Cantilevered M8 shaft; spool loads from +Y end.
# --------------------------------------------------------------------------
TAKEUP_SHAFT_Y = (-90.0, 125.0)     # M8 rod extent (+10 spare thread for wide/small-bore spools)
TAKEUP_TOWER_Y = (-22.0, 0.0)       # bearing tower, two 608s
SPOOL_REF_Y = 5.0                   # back flange of take-up spool sits here (fixed cone shoulder)
FINAL_GEAR_ENVELOPE = dict(r=48.0, y=(-42.0, -22.0))  # 60T fixed gear + hub/lock nut on take-up shaft; WS-A keeps clear
TRAVERSE_TAKEOFF_Y = (-88.0, -80.0) # WS-C mounts its first gear on the take-up shaft here

# --------------------------------------------------------------------------
# Traverse (WS-C). Cross-winding; swappable drum cams per width class.
# --------------------------------------------------------------------------
TRAVERSE_TURNS_PER_STROKE = (8, 12)          # spool turns per one-way stroke
TRAVERSE_STROKES = (36.0, 46.0, 56.0, 64.0)  # cam variants (~ inner spool width - flanges)
TRAVERSE_STROKE_CENTER_Y = (SPOOL_REF_Y + 20.0, SPOOL_REF_Y + 45.0)   # adjustable range
EYELET_XZ = (285.0, 140.0)
CAM_AXIS = (285.0, 112.0)                    # (X, Z), axis parallel to Y
TRAVERSE_TRAIN_Y = (-92.0, -76.0)            # layer for takeoff->cam gears (behind gearbox)
TRAVERSE_BOX = dict(x=(262.0, 308.0), y=(-92.0, 108.0), z=(0.0, 165.0))

# --------------------------------------------------------------------------
# Supply (WS-D). Spool rests on two rollers -> bore independent.
# --------------------------------------------------------------------------
SUPPLY_CENTER_X = 100.0
SUPPLY_ROLLER_DX = 55.0             # rollers at X = center +- this
SUPPLY_ROLLER_Z = 30.0
SUPPLY_ROLLER_D = 26.0
SUPPLY_ROLLER_Y = (-5.0, 95.0)

def supply_spool_center_z(od):
    r = od / 2 + SUPPLY_ROLLER_D / 2
    return SUPPLY_ROLLER_Z + (r * r - SUPPLY_ROLLER_DX ** 2) ** 0.5

# --------------------------------------------------------------------------
# Filament path (WS-E wiper + auto-stop). Fixed points (X, Y, Z).
# Supply spool exit varies (Z ~ 60-260, Y across spool width): funnel needed.
# --------------------------------------------------------------------------
WIPER_INLET = (232.0, SPOOL_CENTER_Y, 140.0)
AUTOSTOP_POINT = (258.0, SPOOL_CENTER_Y, 140.0)
PATH_Z = 140.0
FILAMENT_STATION_BOX = dict(x=(222.0, 268.0), y=(0.0, 90.0), z=(0.0, 165.0))

# --------------------------------------------------------------------------
# Electronics (WS-F): 12 V DC jack, rocker switch, PWM speed module,
# auto-stop microswitch in series with motor. No microcontroller.
# --------------------------------------------------------------------------
ELECTRONICS_BOX = dict(x=(10.0, 100.0), y=(-95.0, -35.0), z=(0.0, 45.0))
PWM_MODULE = (35.0, 25.0, 22.0)     # low-voltage (1.8-15 V) knob PWM board incl. knob height
DC_JACK_D = 11.0                    # 5.5x2.1 panel jack hole
ROCKER_CUTOUT = (19.0, 13.0)        # KCD1 rocker

# Subsystem keep-out envelopes in base coords (x, y, z ranges) for interference checking
ENVELOPES = {
    "gearbox":  dict(x=(300.0, 404.0), y=GEARBOX_Y, z=(0.0, 150.0)),
    "takeup":   dict(x=(355.0, 465.0), y=(-90.0, 130.0), z=(0.0, 245.0)),
    "traverse": [TRAVERSE_BOX,
                 dict(x=(246.0, 421.0), y=TRAVERSE_TRAIN_Y, z=(0.0, 155.0))],   # gear train layer behind the gearbox
    "supply":   dict(x=(30.0, 170.0), y=(-15.0, 105.0), z=(0.0, 60.0)),
    "wiper":    FILAMENT_STATION_BOX,
    "base":     dict(x=BASE_X, y=BASE_Y, z=(-21.0, 45.0)),   # plate + rubber feet + electronics box
}


if __name__ == "__main__":
    print(f"gear ratio {GEAR_RATIO:.1f}:1")
    for n, p in [("takeup", TAKEUP_AXIS), ("S2", SHAFT_S2), ("S1", SHAFT_S1), ("motor", MOTOR_AXIS)]:
        print(f"{n:7s} X={p[0]:7.2f} Z={p[1]:7.2f}")
    for od in (SPOOL_OD_MIN, 200, SPOOL_OD_MAX):
        print(f"supply spool OD {od}: center Z {supply_spool_center_z(od):.1f}")
    print("eyelet->takeup axis", round(hypot(EYELET_XZ[0] - TAKEUP_AXIS[0], EYELET_XZ[1] - TAKEUP_AXIS[1]), 1),
          "(must be > SPOOL_OD_MAX/2 =", SPOOL_OD_MAX / 2, ")")

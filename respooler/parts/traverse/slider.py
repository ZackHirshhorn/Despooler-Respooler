"""Slider (carriage on two Ø6 rods + slotted arm) and the clamp-on 4-roller fairlead.

Modelled in the global frame through axis_y(0, 0, y_ref): local (x, y, z) = (X, -Z, Y - y_ref).
"""
import cadquery as cq

from lib.part import Part, axis_y, loc
from lib.common import cyl
from . import layout as L


def _box(x0, x1, z0, z1, lz0, lz1):
    return (cq.Workplane("XY").center((x0 + x1) / 2, -(z0 + z1) / 2)
            .rect(x1 - x0, z1 - z0).extrude(lz1 - lz0).translate((0, 0, lz0)))


def _vcyl(d, x, lz, z0, z1):
    """Vertical (global Z) cylinder from Z z0 up to z1 at global X x, local z lz."""
    return cq.Workplane("XY").add(cq.Solid.makeCylinder(d / 2, z1 - z0, cq.Vector(x, -z0, lz),
                                                        cq.Vector(0, -1, 0)))


def _ycyl(d, x, z, lz0, lz1):
    return cyl(d, lz1 - lz0, lz0).translate((x, -z, 0))


def slider_shape():
    s = _box(*L.CARR_X, *L.CARR_Z, *L.CARR_DY)
    s = s.union(_box(*L.ARM_X, *L.ARM_Z, L.CARR_DY[0], L.ARM_DY[1]))
    for rx in L.ROD_XS:
        s = s.cut(_ycyl(6.4, rx, L.ROD_Z, L.CARR_DY[0] - 1, L.CARR_DY[1] + 1))
    s = s.cut(_box(L.SLOT_X - 1.7, L.SLOT_X + 1.7, L.ARM_Z[0] - 1, L.ARM_Z[1] + 1, *L.SLOT_DY))
    s = s.cut(_vcyl(L.POST_D + 0.4, L.FX, 0, L.CARR_Z[0] - 1, L.CARR_Z[1] + 1))   # boat pivot
    return s


def fairlead_shape():
    y0, y1 = L.FL_Y
    zc = 140.0
    jaw_z = (118.5, L.ARM_Z[0] - 0.2)
    ped_top = zc - L.ROLLER_L / 2 - 0.3            # 135.7
    wall_top = zc + L.HR_DZ + L.ROLLER_D / 2 + 1.0  # 149.6
    xl = L.VR_X - L.ROLLER_D / 2                    # 270.5
    xr = L.ARM_X[1] + 0.2 + 2.4
    f = _box(xl, xr, *jaw_z, y0, y1)                                       # lower jaw
    f = f.union(_box(xl, L.ARM_X[0] - 0.2, jaw_z[0], ped_top, y0, y1))     # left pedestal
    f = f.union(_box(L.ARM_X[1] + 0.2, xr, jaw_z[0], wall_top, 4.3, y1))   # right cheek
    f = f.union(_box(L.ARM_X[0] - 0.2, xr, L.ARM_Z[1] + 0.2, wall_top, 4.3, y1))   # roller wall
    f = f.union(_box(xl, L.ARM_X[0] - 0.2, ped_top, wall_top, L.VR_DY + L.ROLLER_D / 2 + 0.3, y1))
    # vertical roller axles: M3 inserts in the pedestal
    for dy in (-L.VR_DY, L.VR_DY):
        f = f.cut(_vcyl(4.0, L.VR_X, dy, ped_top - 5.0, ped_top + 1))
        f = f.cut(_vcyl(3.0, L.VR_X, dy, ped_top - 8.0, ped_top))
    # horizontal roller axles: M3 inserts in the wall (along +Y)
    for dz in (-L.HR_DZ, L.HR_DZ):
        f = f.cut(_ycyl(4.0, L.HR_X, zc + dz, 4.0, 9.3))
    # clamp screw: counterbore from the top, through the arm slot into an insert in the jaw
    cy = L.FL_CLAMP_DY
    f = f.cut(_vcyl(6.0, L.SLOT_X, cy, 136.0, wall_top + 1))
    f = f.cut(_vcyl(3.4, L.SLOT_X, cy, jaw_z[1], 136.0))
    f = f.cut(_vcyl(4.0, L.SLOT_X, cy, jaw_z[1] - 5.0, jaw_z[1] + 0.1))
    return f


def roller():
    return cyl(L.ROLLER_D, L.ROLLER_L).cut(cyl(3.4, L.ROLLER_L + 2, -1))


def _screw_v(x, lz, z_head, length, shank=2.9):
    """M3 SHCS pointing down, head bearing face at z_head."""
    return _vcyl(shank, x, lz, z_head - length, z_head).union(_vcyl(5.5, x, lz, z_head, z_head + 3.0))


def build(yf=L.FOLLOW_Y_NOM, ye=L.EYE_Y_NOM):
    off = ye - yf
    assert L.OFFSET_RANGE[0] - 0.01 <= off <= L.OFFSET_RANGE[1] + 0.01, off
    assert L.SLOT_DY[0] + 1.7 <= off + L.FL_CLAMP_DY <= L.SLOT_DY[1] - 1.7
    zc = 140.0
    parts = []
    parts.append(Part("traverse_slider", slider_shape(), loc=axis_y(0, 0, yf),
                      print_loc=loc(0, 0, -L.CARR_DY[0]), color="limegreen",
                      notes="Carriage (printed Ø6.4 bushings on two Ø6 rods) + Ø6.4 boat pivot bore "
                            "+ slotted fairlead arm. Print standing on the carriage rear face (rod "
                            "bores and arm vertical): flat bed face, no supports."))
    parts.append(Part("traverse_fairlead", fairlead_shape(), loc=axis_y(0, 0, ye),
                      print_loc=loc(0, 0, -118.5) * axis_y(0, 0, 0), color="red",
                      notes="4-roller fairlead, 3.2 mm opening. Vertical-axis pair (upstream) loads "
                            "from the top, horizontal-axis pair (downstream) from the -Y side; no "
                            "holes to push filament through. Wraps the arm; one M3x16 through the "
                            "arm slot into an insert in the lower jaw clamps it. Print as installed "
                            "(jaw down); the arm tunnel is a 10.4 mm bridge."))
    for i, dy in enumerate((-L.VR_DY, L.VR_DY)):
        z0 = zc - L.ROLLER_L / 2
        parts.append(Part(f"traverse_roller_v{i}", roller(),
                          loc=loc(L.VR_X, ye + dy, z0), print_loc=loc(), color="white",
                          notes="Printed roller Ø7x8, bore 3.4, on an M3x16 SHCS (vertical axis)"))
        parts.append(Part(f"traverse_roller_v{i}_screw", _screw_v(L.VR_X, dy, z0 + L.ROLLER_L + 0.2, 16),
                          loc=axis_y(0, 0, ye), printed=False, color="gray",
                          notes="M3x16 SHCS + M3 insert (roller axle)"))
    for i, dz in enumerate((-L.HR_DZ, L.HR_DZ)):
        y0r = ye - L.ROLLER_L / 2
        parts.append(Part(f"traverse_roller_h{i}", roller(),
                          loc=axis_y(L.HR_X, zc + dz, y0r), print_loc=loc(), color="white",
                          notes="Printed roller Ø7x8, bore 3.4, on an M3x12 SHCS (axis along Y)"))
        scr = _ycyl(2.9, L.HR_X, zc + dz, -4.3, 7.7).union(_ycyl(5.5, L.HR_X, zc + dz, -7.3, -4.3))
        parts.append(Part(f"traverse_roller_h{i}_screw", scr, loc=axis_y(0, 0, ye), printed=False,
                          color="gray", notes="M3x12 SHCS + M3 insert (roller axle)"))
    parts.append(Part("traverse_fairlead_screw", _screw_v(L.SLOT_X, L.FL_CLAMP_DY, 136.0, 16),
                      loc=axis_y(0, 0, ye), printed=False, color="gray",
                      notes="M3x16 SHCS + M3 insert: fairlead clamp (loosen to centre the stroke)"))
    # boat pivot screw + washer on top of the carriage
    zt = L.CARR_Z[1] + 0.3
    parts.append(Part("traverse_boat_screw",
                      _vcyl(7.0, L.FX, 0, zt, zt + 0.5).cut(_vcyl(3.4, L.FX, 0, zt - 1, zt + 1))
                      .union(_screw_v(L.FX, 0, zt + 0.5, 10, shank=2.4)),
                      loc=axis_y(0, 0, yf), printed=False, color="gray",
                      notes="M3x10 SHCS + M3 washer: boat pivot retainer (self-taps into the post)"))
    return parts

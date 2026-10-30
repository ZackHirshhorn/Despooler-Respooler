# Interface Control Document: Filament Respooler

The integrator owns this document and `params.py`. Workstreams read both and never edit them.
If something here doesn't work for your part, say so in your final report.

## Purpose
This machine rewinds a 1 kg filament spool unattended.
- It moves the bad end of the old spool (tight curl, anchored tail, crossovers) so it no longer reaches the printer first, and clears crossovers.
- It wipes the filament with sponge pads on the way through.
- It stops by itself when the tail end passes.
- It must fit common 1 kg spools: OD 180–230 mm, width 40–80 mm, hub bore 28–62 mm (see `params.py`).

## Global frame (top view, +Z out of page)
```
 Y+ (operator side, free end of take-up shaft)
 ^
 |   [supply spool on rollers]   wiper  stop  eyelet      [take-up spool]
 |        X=100                  232    258   285 (sweeps Y)   X=410
 |------------------------------------------------------------------------> X (filament flow)
 |   [electronics box]                     [traverse train]  [gearbox][tower]
 Y-                                         (Y -92..-76)     (Y -74..-24) (Y -22..0)
```
- X runs along the base and the filament flows toward +X. Y is parallel to all spool and gear axes; +Y is the free end where you load the spool. Z points up, and Z=0 is the top surface of the base.
- The base covers X 0..420 and Y -95..110. It is printed as two halves split at X=210.
- All units are mm.

## Mounting rule (applies to every subsystem)
- The base has an M3 through-hole at every point of a 20 mm grid that starts at (X 5, Y -85). Use `params.grid_points()` and `params.on_grid()`.
- Every foot of every subsystem puts its screw holes **on grid points** and has an M3 heat-set insert at each one (`lib.common.insert_hole("M3")`). The screws go up from underneath the base.
- Feet sit on Z=0.

## Coding contract
- Code in `parts/<subsystem>/` only. The package's `__init__.py` must expose `build() -> list[Part]` and must end with `if __name__ == "__main__": export(build(), "<subsystem>")`. You may split code into extra modules inside your own folder.
- Add a `parts/<subsystem>/__main__.py` too, because `python -m` needs one to run a package. It holds 3 lines: `from . import build`, `from lib.part import export`, `export(build(), "<subsystem>")`.
- Only use colour names CadQuery recognises: `gray`, `gray40`, `gray80`, `lightgray`, `black`, `red`, `green`, `blue`, `orange`. `silver`, `grey` and `dimgray` crash the export.
- Leave the wiring pass-throughs at (55,-62), (315,5) and (235,45) clear, and leave the top grooves along X=335 (Y -24..5) and along Y=45 (X 235..265) clear. If a foot covers one, give the foot a slot for the wire.
- `Part` and the helpers `loc` and `axis_y` are in `lib/part.py`. Model each part in a convenient local frame, set `loc` to place it in the global frame, and set `print_loc` so it sits flat on the bed (zmin = 0) in its print orientation.
- Name parts with your subsystem prefix, e.g. `takeup_cone_back`.
- Include vitamins (bearings, rods, screws that matter for fit, the motor, switches) as `printed=False` parts so the interference check sees them.
- `axis_y(x, z, y0)` maps local +Z to global **+Y**. For a shaft that points -Y, use `loc(x, y0, z, rx=90)`.
- Use `lib/gears.py` (`spur_gear`, `gear_profile`) for every gear, and `lib/common.py` for bearing pockets, inserts, nut traps and clearance holes.
- Run everything from `respooler/` with `../.venv/Scripts/python.exe`:
  - `python -m parts.<subsystem>` exports STEP and STL to `out/`.
  - `python -m checks.run_all <subsystem>` must report no bed-fit problems and no interference within your own subsystem. Envelope warnings must be fixed or explained.
- Each printed part must fit 210×210×200 mm in its print orientation, use walls of at least 2.4 mm, and avoid overhangs steeper than 45° unless they are bridged or explained in notes. The material is PETG.

## Interfaces
| # | Interface | Provider | Consumer | Definition |
|---|---|---|---|---|
| I1 | Motor geometry | vendor | gearbox | `MOTOR` dict and `vendor/motor_130.step` (axis is local +Z). There are **no mount holes**, so the motor needs an embracing clamp that grips the can, keys on the 15.1 mm flats and centres on the 6.15 mm boss. The terminals are on the rear endcap. The front face sits at Y=-55 and the shaft points -Y. |
| I2 | Gear train | gearbox | takeup | 4 stages, 300:1, given by `STAGES`. Shaft positions are `MOTOR_AXIS`, `SHAFT_S1`, `SHAFT_S2`, `SHAFT_S3` and `TAKEUP_AXIS`. The gearbox owns the motor pinion and the compound gears on S1, S2 and S3 (s(n) gear + s(n+1) pinion). **The take-up owns the s4 gear** (60T m1.5, Y -40..-32), which is fixed to the take-up shaft; there is no clutch. Phase is `parts.gearbox.A4G`. |
| I3 | Final gear keep-out | takeup | gearbox | `FINAL_GEAR_ENVELOPE`: a cylinder of R 48 around `TAKEUP_AXIS`, Y -42..-22. The gearbox must not enter it except for the s4 pinion mesh. The take-up shaft passes through the gearbox Y range, so give it an open-top U-slot of at least Ø18. |
| I4 | Take-up shaft | takeup | traverse | An M8 rod at `TAKEUP_AXIS`, spanning `TAKEUP_SHAFT_Y`. It turns at spool speed after the clutch. Traverse mounts its first gear on it within `TRAVERSE_TAKEOFF_Y`, locked with M8 nuts, and needs room for 2 nuts. |
| I5 | Spool reference | takeup | traverse | The back flange of the take-up spool is at Y = `SPOOL_REF_Y` (5). The spool extends toward +Y by 40–80 mm. |
| I6 | Eyelet | traverse | wiper | The filament leaves the auto-stop at `AUTOSTOP_POINT`, travels along +X at Z=140 and enters the eyelet at `EYELET_XZ`. The eyelet sweeps Y; it must be adjustable across `TRAVERSE_STROKE_CENTER_Y`, with stroke chosen by swappable cam (`TRAVERSE_STROKES`). |
| I7 | Supply exit | supply | wiper | The supply spool rests on rollers, so its exit point varies: Z ~60–260, Y anywhere across the spool width. The wiper inlet needs a funnel or roller guide that takes the filament from that range into the line at `WIPER_INLET`. |
| I8 | Auto-stop wiring | wiper | base | The microswitch's 2 wires run down through the station foot into a groove in the base. The switch is wired normally open, held closed while filament presses the lever, and in series with the motor. |
| I9 | Motor wiring | gearbox | base | The 2 motor wires run from the rear endcap down to the base groove. |
| I10 | Electronics | base | all | Lives inside `ELECTRONICS_BOX`: 5–6 V DC jack, rocker switch, low-voltage PWM knob module, a PTC resettable fuse in series with the motor (trips when the motor stalls at the anchored spool tail), and a momentary "thread" button wired in parallel with the auto-stop switch. |

## Subsystem envelopes
See `params.ENVELOPES`. Stay inside yours. If you need more room, request it in your report rather than taking it.

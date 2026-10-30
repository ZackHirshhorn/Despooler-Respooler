# Despooler-Respooler

### EVERYTHING IN THIS REPO EXCEPT FOR THIS LINE WAS DONE BY CLAUDE.AI. I JUST ASKED FOR A FILAMENT REWIND MACHINE

A 3D-printable machine that rewinds a 1 kg filament spool onto an empty spool while you're away. It has three jobs:

- **Fix bad spool endings.** The last few metres of a spool are tightly curled, clipped into the hub, and often have crossed wraps that ruin long prints. Rewinding moves that end to the outside of the new spool, and running the whole spool through clears any crossovers.
- **Wipe the filament.** It passes between two sponge pads on the way through.
- **Stop by itself.** It needs a few seconds of setup and nothing else.

The whole design is parametric Python ([CadQuery](https://github.com/CadQuery/cadquery)). Each subsystem is generated from one shared parameter file and checked automatically for collisions, bed fit and gear mesh.

**Just want to print it?** The STL files, already oriented for printing, are in [`respooler/out/stl/`](respooler/out/stl/). The full assembly is [`respooler/out/step/respooler_assembly.step`](respooler/out/step/respooler_assembly.step). Read [Before you print](#before-you-print) first.

> **Status:** the design is complete and passes every automated check, but it has **not been printed or tested** yet. Expect to tune fits (see [Before you print](#before-you-print)).

## How it works

```
 supply spool    sponge   auto-stop    traverse       take-up spool
 on rollers  ->  wiper -> switch    -> fairlead   ->  on cones
 (any bore)                            (sweeps Y)     driven by 300:1 gearbox
```

| Subsystem | What it does |
|---|---|
| **Supply** | The full spool rests on two rollers, so any hub hole works. Side fences keep it in place, and an adjustable felt brake stops it from unrolling on its own. |
| **Wiper + auto-stop** | A funnel guides the filament into two sponge pads with a weighted lid. Next, a roller-lever microswitch rides on the filament and opens when the tail passes. You thread it by laying filament into open slots. |
| **Traverse** | Gears off the take-up shaft turn a level-wind cam with a crossing groove. A boat-shaped follower in the groove moves a 4-roller fairlead back and forth, cross-winding the spool (about 10 spool turns per stroke). There are four swappable cams for spool widths of 40–80 mm. |
| **Take-up** | A cantilevered M8 shaft runs on two 608 bearings. The back face is flat and the front cone is 45°, so any hub hole from 28 to 62 mm fits, cardboard spools included. |
| **Gearbox** | A 130-size hobby motor sits in an "embracing" clamp (these motors have no mounting holes) and drives a 4-stage printed spur gearbox, 300:1. The spool turns at about 25–35 rpm, so 1 kg takes roughly 30 minutes. |
| **Base + electronics** | The base prints as two halves joined by bolts and dowels. It has a 20 mm grid of M3 holes that every subsystem bolts to. The electronics box holds a 5–6 V jack, a power switch, a PWM speed knob, a PTC fuse and a "thread" button. |

**Stopping:** the machine stops when the free tail passes the auto-stop switch. If the tail is anchored to the old spool, the motor stalls instead and the PTC fuse cuts the power. There is no slip clutch: at 300:1 the motor can't pull hard enough to snap the filament.

## Spool compatibility

Filament spools have no ISO or ASTM standard, so the design covers the range of common 1 kg spools (Prusament, Polymaker, eSun, Bambu, Sunlu, Hatchbox, Overture and others):

| | Supported range |
|---|---|
| Outer diameter | 180–230 mm |
| Width | 40–80 mm |
| Hub hole (take-up spool) | 28–62 mm |
| Hub hole (supply spool) | any (it rests on rollers) |

## Repository layout

```
respooler/
  params.py           single source of truth: every shared dimension and position
  interfaces.md       coordinate frame, mounting rules, interfaces between subsystems
  lib/                gear generator, common features (bearing pockets, inserts...), Part contract
  parts/<subsystem>/  base, gearbox, takeup, traverse, supply, wiper
  assembly.py         builds the full assembly STEP
  checks/             run_all.py (bed fit, envelopes, interference, gears), bom.py
  docs/               BOM.md, print_guide.md, wiring.md
  vendor/             130 motor model (geometry only)
```

## Building the files

Requires Python 3.12 (CadQuery doesn't support the newest Python releases yet). With [uv](https://github.com/astral-sh/uv):

```sh
uv venv --python 3.12 .venv
uv pip install --python .venv/Scripts/python.exe cadquery   # .venv/bin/python on Linux/macOS
cd respooler
../.venv/Scripts/python.exe -m parts.gearbox    # one subsystem -> out/stl, out/step
../.venv/Scripts/python.exe assembly.py         # full assembly -> out/step/respooler_assembly.step
../.venv/Scripts/python.exe -m checks.run_all   # all checks (~80 s)
../.venv/Scripts/python.exe -m checks.bom       # regenerate docs/BOM.md and docs/print_guide.md
```

Ready-made STL and STEP files are included in [`respooler/out/`](respooler/out/), and the generator scripts overwrite them. STL files are already oriented for printing, and every part fits a 220 × 220 mm bed. The traverse export takes about 3 minutes because of the cam groove.

## Before you print

- **Print the gearbox first** and bench-test it at 5–6 V. It is the riskiest part.
- **Tune the bearing fit.** Print a bearing-pocket test and adjust `PRESS_FIT` in `params.py` if the bearings are loose or too tight, then re-export.
- **Tune the PTC fuse.** Start at 0.75 A hold and try sizes between 0.5 and 1.1 A until it trips within seconds of a hard stall but never during normal winding.
- **Check the auto-stop switch.** Make sure the filament tension holds your microswitch closed; a low-force switch is safest.
- See [docs/print_guide.md](respooler/docs/print_guide.md) for settings and orientation, [docs/BOM.md](respooler/docs/BOM.md) for parts, and [docs/wiring.md](respooler/docs/wiring.md) for the wiring.

> **Motor voltage:** the design assumes a 130 motor at **5–6 V maximum**. Never connect a 12 V adapter.

## Changing the design

Change shared dimensions in `params.py`, and change subsystem details inside its own `parts/<name>/` folder. Then run `python -m checks.run_all`. `interfaces.md` describes the mounting grid, the shaft positions and which subsystem owns what.

## License

[MIT](LICENSE)

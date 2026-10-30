"""Collect every subsystem's parts into one assembly. Run from respooler/:
    ../.venv/Scripts/python.exe assembly.py            # all subsystems
    ../.venv/Scripts/python.exe assembly.py gearbox    # just some
"""
import importlib
import pkgutil
import sys
import traceback

import cadquery as cq

import parts as parts_pkg
from lib.part import OUT

SUBSYSTEMS = ["base", "gearbox", "takeup", "traverse", "supply", "wiper"]


def available():
    return [m.name for m in pkgutil.iter_modules(parts_pkg.__path__) if m.ispkg]


def collect(names=None):
    names = names or [n for n in SUBSYSTEMS if n in available()]
    out = {}
    for n in names:
        try:
            out[n] = importlib.import_module(f"parts.{n}").build()
        except Exception:
            print(f"!! {n} failed to build")
            traceback.print_exc()
    return out


def main(names=None):
    built = collect(names)
    asm = cq.Assembly(name="respooler")
    for sub, plist in built.items():
        for p in plist:
            asm.add(p.shape, name=p.name, loc=p.loc, color=cq.Color(p.color))
    (OUT / "step").mkdir(parents=True, exist_ok=True)
    asm.save(str(OUT / "step" / "respooler_assembly.step"))
    print("assembly:", {k: len(v) for k, v in built.items()})
    return built


if __name__ == "__main__":
    main(sys.argv[1:] or None)

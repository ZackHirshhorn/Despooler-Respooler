"""Part contract shared by every workstream.

Each subsystem package parts/<name>/__init__.py exposes:

    def build() -> list[Part]

and ends with:

    if __name__ == "__main__":
        export(build(), "<name>")

so `python -m parts.<name>` (run from respooler/) writes STEP + STL to out/.
"""
from dataclasses import dataclass, field
from pathlib import Path

import cadquery as cq

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "out"


@dataclass
class Part:
    name: str                      # unique, snake_case, prefixed with subsystem e.g. "gearbox_housing_a"
    shape: cq.Workplane            # modeled in its own local frame
    loc: cq.Location = field(default_factory=cq.Location)   # local -> assembly (base) frame
    printed: bool = True           # False for vitamins (bearings, rods, motor, switches)
    qty: int = 1
    print_loc: cq.Location = field(default_factory=cq.Location)  # local -> print-bed orientation
    material: str = "PETG"
    notes: str = ""
    color: str = "orange"

    def placed(self) -> cq.Shape:
        return self.shape.val().moved(self.loc) if isinstance(self.shape, cq.Workplane) else self.shape.moved(self.loc)

    def for_print(self) -> cq.Shape:
        s = self.shape.val() if isinstance(self.shape, cq.Workplane) else self.shape
        return s.moved(self.print_loc)


def loc(x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0) -> cq.Location:
    """Translation then rotations (degrees) about X, Y, Z applied in that order."""
    l = cq.Location(cq.Vector(x, y, z))
    if rx:
        l = l * cq.Location(cq.Vector(), cq.Vector(1, 0, 0), rx)
    if ry:
        l = l * cq.Location(cq.Vector(), cq.Vector(0, 1, 0), ry)
    if rz:
        l = l * cq.Location(cq.Vector(), cq.Vector(0, 0, 1), rz)
    return l


# Axes parallel to global Y: model gears/shafts along local Z, then place with
# axis_y(x, z, y0): local Z -> global +Y, local origin -> (x, y0, z).
def axis_y(x, z, y0) -> cq.Location:
    return loc(x, y0, z, rx=-90)


def export(parts, subsystem):
    (OUT / "step").mkdir(parents=True, exist_ok=True)
    (OUT / "stl").mkdir(parents=True, exist_ok=True)
    asm = cq.Assembly(name=subsystem)
    for p in parts:
        asm.add(p.shape, name=p.name, loc=p.loc, color=cq.Color(p.color))
        if p.printed:
            cq.exporters.export(cq.Workplane().add(p.for_print()), str(OUT / "stl" / f"{p.name}.stl"))
            cq.exporters.export(cq.Workplane().add(p.for_print()), str(OUT / "step" / f"{p.name}.step"))
    asm.save(str(OUT / "step" / f"_asm_{subsystem}.step"))
    print(f"{subsystem}: {len(parts)} parts exported "
          f"({sum(p.printed for p in parts)} printed) -> {OUT}")

"""Traverse (WS-C): purely mechanical cross-winding level wind.

Take-up shaft -> 12T takeoff -> 36T/13T compound idler -> 46T idler -> 21T cam gear
(4.85 spool turns per cam rev) -> diamond (crossing) drum cam, N = 2 cam revs per one-way stroke
(9.7 spool turns per stroke, 19.4 per cycle) -> swivelling boat follower on a slider running on
two Ø6 rods -> clamp-on 4-roller fairlead on the slider's slotted arm.
Cams + boats are swappable (one pair per stroke in params.TRAVERSE_STROKES); the assembly shows 56.
"""
import params as P
from lib.part import export
from . import layout as L
from . import train, cam, frame, slider

ASSEMBLY_STROKE = 56.0


def build():
    parts = []
    parts += frame.build()
    parts += train.build()
    parts += cam.build(ASSEMBLY_STROKE)
    parts += slider.build()
    return parts


def cam_variants():
    """All swappable cams and their matching boats (printed parts)."""
    out = []
    for s in P.TRAVERSE_STROKES:
        out += [cam.cam_part(s), cam.boat_part(s)]
    return out


if __name__ == "__main__":
    export(cam_variants(), "traverse_cams")
    export(build(), "traverse")

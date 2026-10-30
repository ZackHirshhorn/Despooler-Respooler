"""WS-F: base plate (two halves + joint), rubber feet, electronics box and
electrical vitamins. See plate.py / ebox.py and docs/wiring.md."""
from lib.part import export
from parts.base import ebox, plate


def build():
    return [plate.half(1), plate.half(2), plate.joint_hardware(), plate.rubber_feet()] + ebox.build()


if __name__ == "__main__":
    export(build(), "base")

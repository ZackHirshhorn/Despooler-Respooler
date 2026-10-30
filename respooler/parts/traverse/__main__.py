"""Lets `python -m parts.traverse` export (a package runs __main__.py, not __init__)."""
from lib.part import export
from parts.traverse import build, cam_variants

export(cam_variants(), "traverse_cams")
export(build(), "traverse")

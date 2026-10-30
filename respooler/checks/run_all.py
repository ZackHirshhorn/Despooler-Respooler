"""Design checks. Run from respooler/:
    ../.venv/Scripts/python.exe -m checks.run_all [subsystem ...]

1. bed fit       every printed part (print orientation) fits params.BED_MAX
2. envelopes     every placed part stays inside its subsystem's ENVELOPE (warn only)
3. interference  no two placed solids overlap by more than TOL mm^3
                 (except pairs listed in checks/allowed_overlaps.py)
4. gears         stage center distances match shaft positions; ratio report
"""
import sys
from itertools import combinations
from math import hypot

import params as P
from assembly import collect
from checks.allowed_overlaps import ALLOWED

TOL = 1.0  # mm^3


def bb_overlap(a, b, pad=0.01):
    return not (a.xmax < b.xmin - pad or b.xmax < a.xmin - pad or
                a.ymax < b.ymin - pad or b.ymax < a.ymin - pad or
                a.zmax < b.zmin - pad or b.zmax < a.zmin - pad)


def allowed(n1, n2):
    for a, b in ALLOWED:
        if (n1.startswith(a) and n2.startswith(b)) or (n1.startswith(b) and n2.startswith(a)):
            return True
    return False


def check_bed(built):
    bad = []
    for sub, plist in built.items():
        for p in plist:
            if not p.printed:
                continue
            bb = p.for_print().BoundingBox()
            dims = (bb.xlen, bb.ylen, bb.zlen)
            if any(d > m + 1e-6 for d, m in zip(dims, P.BED_MAX)):
                bad.append(f"{p.name}: {tuple(round(d, 1) for d in dims)} > {P.BED_MAX}")
            elif bb.zmin < -0.01 or bb.zmin > 0.01:
                bad.append(f"{p.name}: print orientation not on bed (zmin={bb.zmin:.2f})")
    return bad


def check_envelopes(built):
    warn = []
    for sub, plist in built.items():
        envs = P.ENVELOPES.get(sub)
        if not envs:
            continue
        envs = envs if isinstance(envs, list) else [envs]
        for p in plist:
            if "ghost" in p.name:      # visual reference spools
                continue
            bb = p.placed().BoundingBox()
            rng = {"x": (bb.xmin, bb.xmax), "y": (bb.ymin, bb.ymax), "z": (bb.zmin, bb.zmax)}
            inside = lambda env: all(lo >= env[ax][0] - 0.5 and hi <= env[ax][1] + 0.5 for ax, (lo, hi) in rng.items())
            if not any(inside(e) for e in envs):
                warn.append(f"{p.name}: x {bb.xmin:.1f}..{bb.xmax:.1f} y {bb.ymin:.1f}..{bb.ymax:.1f} "
                            f"z {bb.zmin:.1f}..{bb.zmax:.1f} outside every {sub} envelope")
    return warn


def check_interference(built):
    items = [(p.name, p.placed()) for plist in built.values() for p in plist for _ in [0]]
    boxes = [(n, s, s.BoundingBox()) for n, s in items]
    hits = []
    for (n1, s1, b1), (n2, s2, b2) in combinations(boxes, 2):
        if allowed(n1, n2) or not bb_overlap(b1, b2):
            continue
        try:
            v = s1.intersect(s2).Volume()
        except Exception as e:
            hits.append(f"{n1} x {n2}: boolean failed ({e})")
            continue
        if v > TOL:
            hits.append(f"{n1} x {n2}: {v:.1f} mm^3")
    return hits


def check_gears():
    msgs = []
    axes = [P.MOTOR_AXIS] + [getattr(P, f"SHAFT_S{i}") for i in range(1, len(P.STAGES))] + [P.TAKEUP_AXIS]
    pairs = [(axes[i], axes[i + 1], st) for i, st in enumerate(P.STAGES)]
    for a, b, st in pairs:
        d = hypot(a[0] - b[0], a[1] - b[1])
        cd = P.center_distance(st)
        msgs.append(("OK " if abs(d - cd) < 0.01 else "BAD") +
                    f" {st['name']}: m{st['m']} {st['zp']}/{st['zg']} cd={cd:.2f} actual={d:.2f}")
    msgs.append(f"    gearbox ratio {P.GEAR_RATIO:.1f}:1")
    return msgs


def main(names=None):
    built = collect(names)
    ok = True
    print("== gears");        [print(m) for m in check_gears()]
    bed = check_bed(built)
    print(f"== bed fit ({len(bed)} problems)"); [print("  ", m) for m in bed]
    env = check_envelopes(built)
    print(f"== envelopes ({len(env)} warnings)"); [print("  ", m) for m in env]
    hits = check_interference(built)
    print(f"== interference ({len(hits)} hits)"); [print("  ", m) for m in hits]
    ok = not bed and not hits and all(not m.startswith("BAD") for m in check_gears())
    print("RESULT:", "PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main(sys.argv[1:] or None) else 1)

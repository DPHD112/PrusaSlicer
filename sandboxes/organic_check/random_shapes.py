"""Random figurine-like parts for hunting organic support failures.

    python3 random_shapes.py OUTDIR [COUNT] [SEED]

Each part is a trunk with arms reaching out at different heights, blobs hanging
off the arms, and thin pillars and fins standing under them. The pillars and
fins do not touch the arms, so branches from the arms and blobs must route
around them on the way down. Parts are seeded, so a failing one can be rebuilt.
"""
import sys
from pathlib import Path

import numpy as np

from test_shapes import box, cyl, sphere, to_trimesh


def arm(rng, x0, y0, z0, length, angle, droop, r):
    # A tapered rod from (x0, y0, z0) outwards at `angle`, dropping by `droop` over its length.
    import manifold3d as m3d
    rod = m3d.Manifold.cylinder(float(np.hypot(length, droop)), r, r * rng.uniform(0.5, 1.0), 48)
    tilt = np.degrees(np.arctan2(length, -droop))  # from +Z towards +X
    rod = rod.rotate([0, float(tilt), 0]).rotate([0, 0, float(np.degrees(angle))]).translate([x0, y0, z0])
    tip = np.array([x0, y0, z0]) + np.array([np.cos(angle) * length, np.sin(angle) * length, -droop])
    return rod, tip


def random_part(seed):
    rng = np.random.default_rng(seed)
    height = rng.uniform(30, 55)
    trunk_r = rng.uniform(3, 7)
    part = cyl(trunk_r, height, r_top=trunk_r * rng.uniform(0.6, 1.2), n=64)
    part += sphere(trunk_r * rng.uniform(1.2, 2.0), z=height, n=64)
    obstacles_at = []
    for _ in range(rng.integers(3, 7)):
        z = rng.uniform(0.35, 0.95) * height
        angle = rng.uniform(0, 2 * np.pi)
        length = rng.uniform(10, 30)
        droop = rng.uniform(-8, 12)
        r = rng.uniform(1.2, 3.5)
        rod, tip = arm(rng, 0, 0, z, length, angle, droop, r)
        part += rod
        if rng.random() < 0.7:
            # A blob at the end: a hand, a head, a weapon.
            part += sphere(rng.uniform(2, 6), *tip, n=48)
        if rng.random() < 0.6:
            # A plate hanging off the arm, like a shield or a wing.
            w = rng.uniform(6, 16)
            plate = box(-w / 2, -0.8, -w / 2, w / 2, 0.8, w / 2).rotate([0, 0, float(np.degrees(angle + np.pi / 2))])
            part += plate.translate(list(tip))
        obstacles_at.append((tip, z))
    for tip, z in obstacles_at:
        # Thin features standing under the arm, not touching it, that branches must avoid.
        for _ in range(rng.integers(0, 3)):
            d = rng.uniform(2, 8)
            a = rng.uniform(0, 2 * np.pi)
            x, y = tip[0] + d * np.cos(a), tip[1] + d * np.sin(a)
            top = max(3.0, tip[2] - rng.uniform(4, 15))
            if rng.random() < 0.5:
                part += cyl(rng.uniform(0.8, 2.0), top, x=x, y=y, n=32)
            else:
                t = rng.uniform(0.8, 2.0)
                w = rng.uniform(4, 12)
                fin = box(-w / 2, -t / 2, 0, w / 2, t / 2, top).rotate([0, 0, float(rng.uniform(0, 180))])
                part += fin.translate([x, y, 0])
    # A base plate so thin features stand up, and so the part sits flat.
    base_r = max(12.0, float(np.max([np.hypot(t[0], t[1]) for t, _ in obstacles_at])) + 8)
    part += cyl(base_r * rng.uniform(0.3, 1.0), 2, n=96)
    return part


if __name__ == '__main__':
    out = Path(sys.argv[1] if len(sys.argv) > 1 else '.')
    count = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    first = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    out.mkdir(parents=True, exist_ok=True)
    for seed in range(first, first + count):
        tm = to_trimesh(random_part(seed))
        tm.apply_translation([0, 0, -tm.bounds[0][2]])
        if not tm.is_watertight:
            print(f'random_{seed:03d}: not watertight, skipped')
            continue
        tm.export(out / f'random_{seed:03d}.stl')
        print(f'random_{seed:03d}: {len(tm.faces)} faces, {np.round(tm.extents, 1)} mm')

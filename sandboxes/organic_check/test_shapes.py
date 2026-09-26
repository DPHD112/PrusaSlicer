"""Test parts that make organic supports route around the model on the way down.

    python3 test_shapes.py OUTDIR

Each part is one watertight solid seated on z = 0. They are built so that a
branch leaving an overhang has to dodge a lower feature of the same part, which
is where stock organic supports drop branches that end in mid-air.
"""
import sys
from pathlib import Path

import manifold3d as m3d
import numpy as np
import trimesh


def box(x0, y0, z0, x1, y1, z1):
    return m3d.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl(r, h, z0=0.0, r_top=None, x=0.0, y=0.0, n=96):
    return m3d.Manifold.cylinder(h, r, r if r_top is None else r_top, n).translate([x, y, z0])


def sphere(r, x=0.0, y=0.0, z=0.0, n=96):
    return m3d.Manifold.sphere(r, n).translate([x, y, z])


def roof_over_dome():
    # A roof held up by one corner post, over a dome standing on a base slab.
    # Branches under the roof's middle land on the dome or have to go round it.
    base = box(-30, -30, 0, 30, 30, 2)
    dome = sphere(20, z=2) ^ box(-30, -30, 2, 30, 30, 30)
    post = box(22, 22, 0, 30, 30, 40)
    roof = box(-30, -30, 40, 30, 30, 43)
    return base + dome + post + roof


def mushroom_with_flange():
    # A wide cap on a thin stem, with a flange part-way down the stem. Branches
    # from the cap's underside must pass outside the flange's rim.
    stem = cyl(5, 36)
    flange = cyl(16, 3, z0=12)
    cap = cyl(6, 6, z0=30, r_top=28) + cyl(28, 3, z0=36)
    return stem + flange + cap


def shelf_over_ramp():
    # A shelf sticking out over a ramp that leans out beneath it: the straight
    # drop from the shelf runs into the ramp's steep face.
    wall = box(-5, -20, 0, 5, 20, 45)
    shelf = box(5, -20, 42, 40, 20, 45)
    ramp = m3d.Manifold.extrude(m3d.CrossSection([[[5, 0], [30, 25], [30, 28], [5, 28]]]), 40)
    ramp = ramp.rotate([90, 0, 0]).translate([0, 20, 0])
    return wall + shelf + ramp


def stepped_overhangs():
    # Overhangs stacked above each other at different heights, so branches from
    # the upper ones have to thread past the lower ones.
    core = cyl(6, 50)
    parts = core
    for k, (z, ang) in enumerate([(12, 0), (22, 90), (32, 180), (42, 270)]):
        a = np.radians(ang)
        arm = box(0, -4, 0, 24, 4, 3).rotate([0, 0, ang]).translate([0, 0, z])
        tip = cyl(6, 3, z0=z, x=22 * np.cos(a), y=22 * np.sin(a))
        parts = parts + arm + tip
    return parts


SHAPES = {
    'roof_over_dome': roof_over_dome,
    'mushroom_with_flange': mushroom_with_flange,
    'shelf_over_ramp': shelf_over_ramp,
    'stepped_overhangs': stepped_overhangs,
}


def to_trimesh(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=False)


if __name__ == '__main__':
    out = Path(sys.argv[1] if len(sys.argv) > 1 else '.')
    out.mkdir(parents=True, exist_ok=True)
    for name, make in SHAPES.items():
        tm = to_trimesh(make())
        tm.apply_translation([0, 0, -tm.bounds[0][2]])
        assert tm.is_watertight, name
        tm.export(out / f'{name}.stl')
        print(f'{name}: {len(tm.faces)} faces, {np.round(tm.extents, 1)} mm')

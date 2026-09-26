"""Find support islands in a G-code file that start with nothing under them.

    python3 check_floating.py print.gcode [--px 0.1] [--below 0.6] [--min-area 0.3]

Every support island printed at height z has to sit on something: the bed, the
part, or support printed just below it. This reads the extrusions per layer from
PrusaSlicer's ;TYPE: / ;WIDTH: / ;Z: comments, rasterizes them, and reports each
support island whose footprint has (almost) no plastic under it within `--below`
mm. That window has to span the support's bottom contact gap, or islands resting
on the part through that gap would be reported too.

Exit code 1 when any floating island is found, so it can gate a test.
"""
import argparse
import re
import sys
from collections import defaultdict

import numpy as np
from scipy import ndimage

SUPPORT_TYPES = ('Support material', 'Support material interface')


def parse(path):
    """Return {z: [(x0, y0, x1, y1, width, is_support), ...]} of extrusion moves."""
    layers = defaultdict(list)
    z = 0.0
    width = 0.45
    kind = ''
    x = y = 0.0
    relative_e = True
    e_abs = 0.0
    num = re.compile(r'([XYZEF])(-?\d*\.?\d+)')
    with open(path, errors='replace') as f:
        for line in f:
            if line.startswith(';Z:'):
                z = round(float(line[3:]), 4)
            elif line.startswith(';TYPE:'):
                kind = line[6:].strip()
            elif line.startswith(';WIDTH:'):
                width = float(line[7:])
            elif line.startswith('M82'):
                relative_e = False
            elif line.startswith('M83'):
                relative_e = True
            elif line.startswith('G92'):
                v = dict(num.findall(line.split(';')[0]))
                if 'E' in v:
                    e_abs = float(v['E'])
            elif line.startswith(('G1 ', 'G0 ', 'G2 ', 'G3 ')):
                v = {k: float(s) for k, s in num.findall(line.split(';')[0])}
                nx, ny = v.get('X', x), v.get('Y', y)
                extruding = False
                if 'E' in v:
                    if relative_e:
                        extruding = v['E'] > 0
                    else:
                        extruding = v['E'] > e_abs
                        e_abs = v['E']
                if extruding and (nx != x or ny != y) and kind not in ('Wipe tower', 'Custom'):
                    layers[z].append((x, y, nx, ny, width, kind in SUPPORT_TYPES))
                x, y = nx, ny
    return dict(sorted(layers.items()))


def rasterize(segments, origin, shape, px, support_only):
    grid = np.zeros(shape, bool)
    ox, oy = origin
    for x0, y0, x1, y1, w, sup in segments:
        if support_only and not sup:
            continue
        n = max(2, int(np.hypot(x1 - x0, y1 - y0) / (px * 0.5)) + 1)
        xs = np.linspace(x0, x1, n)
        ys = np.linspace(y0, y1, n)
        r = max(1, int(round(w / (2 * px))))
        for cx, cy in zip(((xs - ox) / px).astype(int), ((ys - oy) / px).astype(int)):
            grid[max(cy - r, 0):cy + r + 1, max(cx - r, 0):cx + r + 1] = True
    return grid


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('gcode')
    ap.add_argument('--px', type=float, default=0.1, help='raster pixel size, mm')
    ap.add_argument('--below', type=float, default=0.6, help='how far below an island to look for plastic, mm')
    ap.add_argument('--min-area', type=float, default=0.3, help='ignore islands smaller than this, mm^2')
    ap.add_argument('--min-held', type=float, default=0.10, help='an island counts as held when this fraction of it has plastic below')
    ap.add_argument('--sparse-gap', type=float, default=3.0,
                    help='support below counts as a solid area across gaps up to this wide, mm (sparse support infill)')
    args = ap.parse_args()

    layers = parse(args.gcode)
    if not layers:
        sys.exit('no extrusions found (is this PrusaSlicer G-code with ;Z: comments?)')
    pts = np.array([(s[0], s[1]) for segs in layers.values() for s in segs] +
                   [(s[2], s[3]) for segs in layers.values() for s in segs])
    origin = pts.min(0) - 2
    shape = tuple((((pts.max(0) + 2) - origin) / args.px).astype(int)[::-1] + 1)
    zs = list(layers)
    first_z = zs[0]

    all_grids = {}
    support_grids = {}
    closing_r = max(1, int(round(args.sparse_gap / 2 / args.px)))
    yy, xx = np.mgrid[-closing_r:closing_r + 1, -closing_r:closing_r + 1]
    closing = xx * xx + yy * yy <= closing_r * closing_r
    floating = []
    n_support_layers = 0
    for z in zs:
        all_grids[z] = rasterize(layers[z], origin, shape, args.px, support_only=False)
        sup = rasterize(layers[z], origin, shape, args.px, support_only=True)
        # Support printed as sparse lines holds up the layer above across the gaps between them.
        support_grids[z] = ndimage.binary_closing(sup, closing) if sup.any() else sup
        if not sup.any():
            continue
        n_support_layers += 1
        if z <= first_z + 1e-6:
            continue  # on the bed
        below = np.zeros(shape, bool)
        for zb in zs:
            if z - args.below - 1e-6 <= zb < z - 1e-6:
                below |= all_grids[zb] | support_grids[zb]
        below = ndimage.binary_dilation(below, iterations=1)
        labels, n = ndimage.label(sup)
        if n == 0:
            continue
        idx = np.arange(1, n + 1)
        sizes = ndimage.sum(np.ones_like(labels), labels, idx)
        held = ndimage.sum(below, labels, idx)
        for i, size, h in zip(idx, sizes, held):
            area = size * args.px ** 2
            if area < args.min_area or h / size >= args.min_held:
                continue
            cy, cx = ndimage.center_of_mass(labels == i)
            floating.append((z, origin[0] + cx * args.px, origin[1] + cy * args.px, area, h / size))

    print(f'{args.gcode}: {len(zs)} layers, {n_support_layers} with support')
    if not floating:
        print('OK: every support island has plastic under it')
        return 0
    print(f'FLOATING: {len(floating)} support islands with nothing under them')
    for z, x, y, area, frac in floating[:40]:
        print(f'  z={z:7.2f}  at ({x:7.2f}, {y:7.2f})  area {area:6.2f} mm2  held {frac:4.0%}')
    if len(floating) > 40:
        print(f'  ... and {len(floating) - 40} more')
    return 1


if __name__ == '__main__':
    sys.exit(main())

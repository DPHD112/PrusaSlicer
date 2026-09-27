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
    """Mark every pixel within half a line width (as a square) of the sampled extrusion paths."""
    grid = np.zeros(shape, bool)
    seg = np.array([s[:5] for s in segments if s[5] or not support_only], float).reshape(-1, 5)
    if len(seg) == 0:
        return grid
    x0, y0, x1, y1, w = seg.T
    # Sample each segment every half pixel.
    n = np.maximum(2, (np.hypot(x1 - x0, y1 - y0) / (px * 0.5)).astype(int) + 1)
    idx = np.repeat(np.arange(len(seg)), n)
    starts = np.cumsum(n) - n
    t = (np.arange(n.sum()) - np.repeat(starts, n)) / np.repeat(n - 1, n)
    cx = ((x0[idx] + t * (x1 - x0)[idx] - origin[0]) / px).astype(int)
    cy = ((y0[idx] + t * (y1 - y0)[idx] - origin[1]) / px).astype(int)
    r = np.maximum(1, np.round(w / (2 * px)).astype(int))[idx]
    inside = (cx >= 0) & (cx < shape[1]) & (cy >= 0) & (cy < shape[0])
    for radius in np.unique(r[inside]):
        sel = inside & (r == radius)
        centers = np.zeros(shape, bool)
        centers[cy[sel], cx[sel]] = True
        grid |= ndimage.maximum_filter(centers, size=2 * int(radius) + 1)
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
    closing_r = max(1.0, args.sparse_gap / 2 / args.px)

    def close_gaps(grid):
        # Morphological closing with a disk, done with distance transforms (much faster than
        # binary_closing with a large structuring element), on the support's bounding box only.
        ys, xs = np.nonzero(grid)
        pad = int(np.ceil(closing_r)) + 2
        y0, y1 = max(ys.min() - pad, 0), min(ys.max() + pad + 1, grid.shape[0])
        x0, x1 = max(xs.min() - pad, 0), min(xs.max() + pad + 1, grid.shape[1])
        crop = grid[y0:y1, x0:x1]
        grown = ndimage.distance_transform_edt(~crop) <= closing_r
        out = np.zeros_like(grid)
        out[y0:y1, x0:x1] = ndimage.distance_transform_edt(grown) > closing_r
        return out
    floating = []
    n_support_layers = 0
    support_rasters = {}

    def closed_support(zb):
        # Only needed for islands that look unheld, so computed on demand.
        if zb not in support_grids:
            sup_b = support_rasters[zb]
            support_grids[zb] = close_gaps(sup_b) if sup_b.any() else sup_b
        return support_grids[zb]

    for z in zs:
        all_grids[z] = rasterize(layers[z], origin, shape, args.px, support_only=False)
        sup = rasterize(layers[z], origin, shape, args.px, support_only=True)
        support_rasters[z] = sup
        if not sup.any():
            continue
        n_support_layers += 1
        if z <= first_z + 1e-6:
            continue  # on the bed
        window = [zb for zb in zs if z - args.below - 1e-6 <= zb < z - 1e-6]
        below = np.zeros(shape, bool)
        for zb in window:
            below |= all_grids[zb]
        below = ndimage.binary_dilation(below, iterations=1)
        labels, n = ndimage.label(sup)
        if n == 0:
            continue
        sizes = np.bincount(labels.ravel(), minlength=n + 1)
        held = np.bincount(labels.ravel(), weights=below.ravel(), minlength=n + 1)
        suspects = [i for i in range(1, n + 1)
                    if sizes[i] * args.px ** 2 >= args.min_area and held[i] / sizes[i] < args.min_held]
        if not suspects:
            continue
        # Support printed as sparse lines holds up the layer above across the gaps between them.
        for zb in window:
            below |= ndimage.binary_dilation(closed_support(zb), iterations=1)
        held = np.bincount(labels.ravel(), weights=below.ravel(), minlength=n + 1)
        for i in suspects:
            size, h = sizes[i], held[i]
            if h / size >= args.min_held:
                continue
            cy, cx = ndimage.center_of_mass(labels == i)
            floating.append((z, origin[0] + cx * args.px, origin[1] + cy * args.px, size * args.px ** 2, h / size))

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

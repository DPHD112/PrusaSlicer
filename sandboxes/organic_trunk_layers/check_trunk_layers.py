"""Check organic supports printed with thick trunk layers against a normal slice.

    python3 check_trunk_layers.py trunk.gcode [--baseline normal.gcode] [--px 0.1]

Reads PrusaSlicer G-code (;Z:, ;HEIGHT:, ;WIDTH:, ;TYPE: comments) and reports:

- how much of the support is printed at each layer height, and how many layers
  print only support;
- the print time from the slicer's estimate, and a rough time spent on support
  (length / feed rate, so without acceleration);
- the smallest gap between support and part extrusions that overlap in Z, since
  thick layers must not bring the trunks closer to the part;
- support islands with no plastic under them;
- with --baseline, the tips: wherever the normal slice has support ending right
  under the part (within --tip-gap, the contact distance plus a layer), the
  support must end at the same height, and its last --tip-depth mm must use
  layers no thicker than the part's. Support ending further below the part,
  like the side of a branch passing under a sloped overhang, does not touch it.

Exit code 1 when a support island floats, the gap to the part shrinks (below
the baseline's, or below --min-gap without a baseline), or (with --baseline) a
tip moves, goes missing or is printed with thick layers.
"""
import argparse
import re
import sys
from collections import defaultdict

import numpy as np
from scipy import ndimage

SUPPORT_TYPES = ('Support material', 'Support material interface')


def parse(path, times=None, estimate=None):
    """Return a list of layers (z, [(x0, y0, x1, y1, width, height, is_support), ...]) sorted by z.

    With times, also add up length / feed rate per extrusion type (and 'Travel') into it.
    With estimate, put the slicer's estimated print time there as estimate['time'].
    """
    layers = defaultdict(list)
    feed = 1.0
    z = 0.0
    width = 0.45
    height = 0.2
    kind = ''
    x = y = 0.0
    relative_e = True
    e_abs = 0.0
    num = re.compile(r'([XYZEF])(-?\d*\.?\d+)')
    with open(path, errors='replace') as f:
        for line in f:
            if estimate is not None and line.startswith('; estimated printing time (normal mode)'):
                estimate['time'] = line.split('=', 1)[1].strip()
            if line.startswith(';Z:'):
                z = round(float(line[3:]), 4)
            elif line.startswith(';HEIGHT:'):
                height = float(line[8:])
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
                feed = v.get('F', feed)
                extruding = False
                if 'E' in v:
                    if relative_e:
                        extruding = v['E'] > 0
                    else:
                        extruding = v['E'] > e_abs
                        e_abs = v['E']
                if extruding and (nx != x or ny != y) and kind not in ('Wipe tower', 'Custom'):
                    layers[z].append((x, y, nx, ny, width, height, kind in SUPPORT_TYPES))
                if times is not None and (nx != x or ny != y):
                    times[kind if extruding else 'Travel'] += np.hypot(nx - x, ny - y) / (feed / 60.)
                x, y = nx, ny
    return sorted(layers.items())


class Raster:
    def __init__(self, layers_list, px):
        pts = np.array([(s[0], s[1]) for _, segs in layers_list for s in segs] +
                       [(s[2], s[3]) for _, segs in layers_list for s in segs])
        self.px = px
        self.origin = pts.min(0) - 2
        self.shape = tuple((((pts.max(0) + 2) - self.origin) / px).astype(int)[::-1] + 1)

    def draw(self, segments):
        grid = np.zeros(self.shape, bool)
        ox, oy = self.origin
        px = self.px
        for x0, y0, x1, y1, w in segments:
            n = max(2, int(np.hypot(x1 - x0, y1 - y0) / (px * 0.5)) + 1)
            xs = ((np.linspace(x0, x1, n) - ox) / px).astype(int)
            ys = ((np.linspace(y0, y1, n) - oy) / px).astype(int)
            r = max(0, int(round(w / (2 * px))) - 1)
            for cx, cy in zip(xs, ys):
                grid[max(cy - r, 0):cy + r + 1, max(cx - r, 0):cx + r + 1] = True
        return grid


def slabs(layers_list, raster):
    """Split each G-code layer into support and part slabs: (bottom_z, top_z, grid, is_support).

    Support slabs start at z minus their ;HEIGHT:, one per height. A part slab starts at the
    previous part layer, as bridges report the height of their flow, not of the layer.
    """
    out = []
    part_z = 0.0
    for z, segs in layers_list:
        by_height = defaultdict(list)
        part = []
        for s in segs:
            if s[6]:
                by_height[round(s[5], 3)].append(s[:5])
            else:
                part.append(s[:5])
        for h, support in by_height.items():
            out.append((round(z - h, 4), z, raster.draw(support), True))
        if part:
            out.append((part_z, z, raster.draw(part), False))
            part_z = z
    return out


def smallest_gap(support_slabs, part_slabs, raster):
    """Return (gap, z0, z1): the smallest XY gap between support and part extrusions overlapping in Z."""
    best = (np.inf, None, None)
    for z0, z1, grid, _ in support_slabs:
        part = np.zeros(raster.shape, bool)
        for p0, p1, pgrid, _ in part_slabs:
            if p0 < z1 - 1e-4 and p1 > z0 + 1e-4:
                part |= pgrid
        if not part.any() or not grid.any():
            continue
        dist = ndimage.distance_transform_edt(~part) * raster.px
        gap = dist[grid].min() - raster.px
        if gap < best[0]:
            best = (gap, z0, z1)
    return best


def height_maps(support_slabs, shape):
    top = np.full(shape, -np.inf)
    bottom = np.full(shape, np.inf)
    for z0, z1, grid, _ in support_slabs:
        top[grid] = np.maximum(top[grid], z1)
        bottom[grid] = np.minimum(bottom[grid], z0)
    return top, bottom


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('gcode')
    ap.add_argument('--baseline', help='the same part sliced without thick trunk layers')
    ap.add_argument('--px', type=float, default=0.1, help='raster pixel size, mm')
    ap.add_argument('--below', type=float, default=0.6, help='how far below a support island to look for plastic, mm')
    ap.add_argument('--min-area', type=float, default=0.3, help='ignore floating islands smaller than this, mm^2')
    ap.add_argument('--min-gap', type=float, default=0.05, help='fail when support comes closer to the part than this, mm')
    ap.add_argument('--tip-gap', type=float, default=0.5,
                    help='support ending at most this far below the part is a tip: the contact distance (0.2 mm by default) plus a layer, mm')
    ap.add_argument('--tip-depth', type=float, default=1.0, help='how deep below a tip the layers must stay thin, mm')
    args = ap.parse_args()

    times, estimate = defaultdict(float), {}
    layers_list = parse(args.gcode, times, estimate)
    if not layers_list:
        sys.exit('no extrusions found (is this PrusaSlicer G-code with ;Z: comments?)')
    base_times, base_estimate = defaultdict(float), {}
    base_list = parse(args.baseline, base_times, base_estimate) if args.baseline else None
    raster = Raster(layers_list + (base_list or []), args.px)
    failed = False

    # Support by layer height.
    length = defaultdict(float)
    support_only = 0
    for z, segs in layers_list:
        if segs and all(s[6] for s in segs):
            support_only += 1
        for x0, y0, x1, y1, w, h, sup in segs:
            if sup:
                length[round(h, 3)] += np.hypot(x1 - x0, y1 - y0)
    total = sum(length.values())
    print(f'{args.gcode}: {len(layers_list)} layers, {support_only} print only support')
    for h in sorted(length):
        print(f'  support at {h:.3f} mm layers: {length[h] / 1000:8.2f} m ({length[h] / max(total, 1e-9):5.1%})')

    def support_minutes(t):
        return sum(t[k] for k in SUPPORT_TYPES) / 60.
    line = f'  print time estimate: {estimate.get("time", "not in the G-code")}'
    if base_list:
        line += f' (baseline {base_estimate.get("time", "not in the G-code")})'
    print(line)
    line = f'  support extrusion, length / feed rate: {support_minutes(times):.1f} min'
    if base_list:
        line += f' (baseline {support_minutes(base_times):.1f} min)'
    print(line)

    all_slabs = slabs(layers_list, raster)
    support_slabs = [s for s in all_slabs if s[3]]
    part_slabs = [s for s in all_slabs if not s[3]]

    base_slabs = slabs(base_list, raster) if base_list else None
    base_support = [s for s in base_slabs if s[3]] if base_list else None
    base_part = [s for s in base_slabs if not s[3]] if base_list else None

    # Smallest gap between support and part extrusions overlapping in Z.
    # The first layer is left out: the support's first layer does not change, and it is usually the closest.
    first_z = layers_list[0][0]
    above_first = lambda slab_list: [s for s in slab_list if s[1] > first_z + 1e-4]
    gap, z0, z1 = smallest_gap(above_first(support_slabs), part_slabs, raster)
    if z0 is not None:
        line = f'  smallest gap between support and part above the first layer: {max(gap, 0):.2f} mm (support from z={z0:.2f} to {z1:.2f})'
        base_gap = smallest_gap(above_first(base_support), base_part, raster)[0] if base_list else np.inf
        if base_list:
            line += f', baseline {max(base_gap, 0):.2f} mm'
        print(line)
        if gap < base_gap - 2 * args.px:
            print('  FAIL: support closer to the part than in the baseline')
            failed = True
        elif gap < args.min_gap and not base_list:
            print(f'  FAIL: support closer to the part than {args.min_gap} mm')
            failed = True

    # Support islands with nothing under them.
    floating = []
    for z0, z1, grid, _ in support_slabs:
        if z0 <= 1e-4 or z1 <= first_z + 1e-4:
            continue
        below = np.zeros(raster.shape, bool)
        for b0, b1, bgrid, _ in all_slabs:
            if z0 - args.below - 1e-4 <= b1 <= z0 + 1e-4:
                below |= bgrid
        below = ndimage.binary_dilation(below, iterations=1)
        labels, n = ndimage.label(grid)
        if n == 0:
            continue
        idx = np.arange(1, n + 1)
        sizes = ndimage.sum(np.ones_like(labels), labels, idx)
        held = ndimage.sum(below, labels, idx)
        for i, size, h in zip(idx, sizes, held):
            if size * args.px ** 2 >= args.min_area and h / size < 0.1:
                floating.append((z1, size * args.px ** 2))
    if floating:
        print(f'  FAIL: {len(floating)} support islands with nothing under them, first at z={floating[0][0]:.2f}')
        failed = True
    else:
        print('  every support island has plastic under it')

    if base_list:
        top, _ = height_maps(support_slabs, raster.shape)
        btop, _ = height_maps(base_support, raster.shape)
        # Tips: where the baseline support ends right under the part.
        tip = np.zeros(raster.shape, bool)
        for p0, p1, pgrid, _ in part_slabs:
            tip |= pgrid & (p0 >= btop - 1e-3) & (p0 <= btop + args.tip_gap)
        px2 = args.px ** 2
        missing = tip & ~np.isfinite(top)
        present = tip & np.isfinite(top)
        dtop = np.abs(top[present] - btop[present])
        moved = (dtop > 0.01).sum() * px2
        print(f'  tips (support ending under the part): {tip.sum() * px2:.1f} mm2, moved on {moved:.2f} mm2'
              f' (max {dtop.max() if dtop.size else 0:.2f} mm), missing on {missing.sum() * px2:.2f} mm2')
        # Thick layers in the last tip_depth mm below a tip.
        thin = min((round(s[1] - s[0], 3) for s in part_slabs), default=0.2)
        thick_near_tip = np.zeros(raster.shape, bool)
        for s0, s1, grid, _ in support_slabs:
            if s1 - s0 > thin + 1e-3:
                thick_near_tip |= grid & tip & (s1 > top - args.tip_depth + 1e-3)
        print(f'  tips with layers thicker than {thin} mm in their last {args.tip_depth} mm: {thick_near_tip.sum() * px2:.2f} mm2')
        # Tiny areas along the outlines come from rasterizing slightly different toolpaths.
        if moved > 1.0 or missing.sum() * px2 > 1.0 or thick_near_tip.sum() * px2 > 1.0:
            print('  FAIL: the tips changed')
            failed = True
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())

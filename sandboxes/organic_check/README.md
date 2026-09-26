# Organic support checks

Tools for checking that organic (tree) supports keep clear of the part on the way
down and come out as solid trees, with no islands starting in mid-air.

- `test_shapes.py OUTDIR` writes test parts that force branches to route around
  lower features of the same part (roof over a dome, mushroom with a flange,
  shelf over a ramp, stepped arms).
- `random_shapes.py OUTDIR [COUNT] [SEED]` writes seeded random figurine-like parts
  (a trunk with arms, blobs and plates, and thin pillars and fins under them) for
  hunting failures on shapes nobody designed.
- `organic_check part.stl part.gcode [layer_height] [key=value ...]` slices one STL
  with organic supports and default settings, without the GUI. `key=value` changes
  a print setting, such as `support_material_threshold=55` or
  `support_material_buildplate_only=1`. It is built with the tests
  (`SLIC3R_BUILD_TESTS`).
- `check_floating.py print.gcode` rasterizes each layer of a PrusaSlicer G-code
  and reports support islands with no plastic below them. Exit code 1 if any.

Slice each shape, then run the checker on the G-code:

    pip install numpy scipy trimesh manifold3d
    python3 test_shapes.py shapes
    PRUSASLICER_ORGANIC_STATS=1 organic_check shapes/roof_over_dome.stl shapes/roof_over_dome.gcode
    python3 check_floating.py shapes/roof_over_dome.gcode

Environment variables, read while slicing (by `organic_check` or the slicer):

- `PRUSASLICER_ORGANIC_STATS=1` prints how many branches came within the XY or Z
  gap of the part and had to be clipped, and the clipped volume, both in total and
  away from branch tips and roots (the second is branches running into the part on
  their way down); plus how many floating islands were extended down or removed.
- `PRUSASLICER_EXPORT_ORGANIC_STL=supports.stl` writes the support as it will be
  printed (final per-layer areas stacked into a solid), placed on the bed like the
  G-code, to import into another slicer next to the part. The mesh is closed apart
  from a few hundred sliver edges out of several hundred thousand, which slicers
  repair on import.
- `PRUSASLICER_EXPORT_ORGANIC_TUBES_STL=tubes.stl` writes the raw smooth branch
  tubes before they are clipped against the part, for debugging.

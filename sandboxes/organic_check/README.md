# Organic support checks

Tools for checking that organic (tree) supports keep clear of the part on the way
down and come out as solid trees, with no islands starting in mid-air.

- `test_shapes.py OUTDIR` writes test parts that force branches to route around
  lower features of the same part (roof over a dome, mushroom with a flange,
  shelf over a ramp, stepped arms).
- `organic_check part.stl part.gcode [layer_height]` slices one STL with organic
  supports and default settings, without the GUI. It is built with the tests
  (`SLIC3R_BUILD_TESTS`).
- `check_floating.py print.gcode` rasterizes each layer of a PrusaSlicer G-code
  and reports support islands with no plastic below them. Exit code 1 if any.

Slice each shape, then run the checker on the G-code:

    pip install numpy scipy trimesh manifold3d
    python3 test_shapes.py shapes
    PRUSASLICER_ORGANIC_STATS=1 organic_check shapes/roof_over_dome.stl shapes/roof_over_dome.gcode
    python3 check_floating.py shapes/roof_over_dome.gcode

Environment variables, read while slicing (by `organic_check` or the slicer):

- `PRUSASLICER_ORGANIC_STATS=1` prints how many branches ran into the part (and
  had to be clipped), the clipped volume, and how many floating islands were
  extended down or removed.
- `PRUSASLICER_EXPORT_ORGANIC_STL=supports.stl` writes the support as it will be
  printed (final per-layer areas stacked into a solid), placed on the bed like the
  G-code, to import into another slicer next to the part.
- `PRUSASLICER_EXPORT_ORGANIC_TUBES_STL=tubes.stl` writes the raw smooth branch
  tubes before they are clipped against the part, for debugging.

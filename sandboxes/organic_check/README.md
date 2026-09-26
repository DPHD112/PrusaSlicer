# Organic support checks

Tools for catching organic (tree) support islands that start in mid-air.

- `test_shapes.py OUTDIR` writes test parts that force branches to route around
  lower features of the same part (roof over a dome, mushroom with a flange,
  shelf over a ramp, stepped arms).
- `check_floating.py print.gcode` rasterizes each layer of a PrusaSlicer G-code
  and reports support islands with no plastic below them. Exit code 1 if any.

Slice each shape with organic supports, then run the checker on the G-code:

    pip install numpy scipy trimesh manifold3d
    python3 test_shapes.py shapes
    prusa-slicer --export-gcode --support-material --support-material-style organic \
        --output shapes/roof_over_dome.gcode shapes/roof_over_dome.stl
    python3 check_floating.py shapes/roof_over_dome.gcode

Set `PRUSASLICER_EXPORT_ORGANIC_STL=/path/supports.stl` while slicing to also
write the organic branch tubes as a mesh, positioned on the bed like the G-code.

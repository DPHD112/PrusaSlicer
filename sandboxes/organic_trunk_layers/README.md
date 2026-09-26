# Thick trunk layers for organic supports

Prototype: print the trunks of organic (tree) supports with thicker layers than
the part, for example 0.3 mm trunks next to 0.2 mm part layers, while the top of
every branch keeps the part's layer height so the tips meet the part exactly as
before.

It is off by default. Turn it on with Trunk Layer Height in the Organic
supports group of the support settings (`support_tree_trunk_layer_height`,
in mm, 0 is off), for example 0.3 for a part printed at 0.2 mm.

## How it works

`organic_thick_trunk_layers()` in `src/libslic3r/src/libslic3r/Support/TreeSupport.cpp`
runs after the tree support areas are final, before they are turned into support
layers.

- It picks the thickest trunk layer height H = m * h / n that fits under both the
  requested height and the extruder's maximum layer height (`max_layer_height`,
  0 means 75% of the nozzle), so that m part layers of height h equal n thick
  layers. 0.2 mm parts with 0.3 mm trunks give m = 3, n = 2: the two grids meet
  every 0.6 mm.
- The part layers above the first layer are grouped into blocks of m layers.
  In each block, an island of support becomes a thick trunk only if:
  - it stands on support right below the block, leaning no more than the
    preferred branch angle allows;
  - no branch ends inside the block (no tip, no merge that stops);
  - all of it continues upwards for another `tip_layers + 2` layers above the
    block (7 layers, 1.4 mm, at the default branch sizes and 0.2 mm layers), so
    every tip keeps the part's layer height;
  - no interface, top contact or bottom contact layer comes near it, in the
    block or in these layers above it;
  - the part is not right above it in these layers, nor within the top contact
    distance above them. A branch running under a sloped overhang touches the
    part along its side, far from its own tip; this keeps the fine layers there
    too.
- The trunk islands move from the part-height layers to n new thick layers. Each
  thick layer is clipped again against the part's slices at every part layer it
  spans, as the original areas were only clipped at their own height.
- `generate_support_toolpaths()` in `SupportCommon.cpp` prints the thick layers
  as organic tubes with the thicker flow.

Every other island stays on the part's layers. The first layer, the raft,
soluble interfaces, variable layer height and prints with a wipe tower are left
alone.

Between part layers (at 0.5, 1.1, ... mm for 0.2/0.3) the printer prints support
only, like classic supports with a support layer height above the part's.

## Checking a slice

`check_trunk_layers.py` compares a G-code sliced with thick trunks to the same
part sliced without them:

    slice_stl part.stl trunk.gcode support_tree_trunk_layer_height=0.3
    slice_stl part.stl normal.gcode
    python3 check_trunk_layers.py trunk.gcode --baseline normal.gcode

It reports how much support is printed at each layer height, the print time,
the smallest gap between support and part (thick layers must not come closer),
support islands with nothing under them, and the tips: wherever the normal slice
has support ending within 0.5 mm below the part (the contact distance plus a
layer), the support must end at the same height and its last 1 mm
(`--tip-depth`) must use the part's layer height. Exit code 1 on any failure.

## Results on synthetic shapes

The four test shapes from `sandboxes/organic_check/test_shapes.py` (branch
organic-no-floating), sliced with `slice_stl` at 0.2 mm layers and 0.3 mm
trunks, with its defaults otherwise (organic supports everywhere, 0.3 mm XY
spacing, PrusaSlicer's defaults for the rest):

| Shape                | Support at 0.3 mm | Print time estimate     | Support extrusion |
|----------------------|------------------:|-------------------------|-------------------|
| mushroom_with_flange |             45.6% | 2h 54m 14s → 2h 37m 28s | 40.3 → 33.2 min   |
| roof_over_dome       |             77.6% | 3h 2m 25s → 2h 50m 18s  | 20.0 → 14.7 min   |
| shelf_over_ramp      |             80.4% | 1h 44m 33s → 1h 38m 2s  | 10.2 → 7.4 min    |
| stepped_overhangs    |             65.8% | 1h 7m 49s → 1h 1m 31s   | 10.3 → 7.9 min    |

On all four, `check_trunk_layers.py` passes: the support ends at the same
height under the part, the smallest gap to the part is unchanged, no support
floats, and every tip keeps at least 1.2 mm of 0.2 mm layers under it. With
`--tip-depth 1.4`, only 3 mm² of the mushroom's 1259 mm² of tips, under its
sloped cap, have less. 53 to 65 layers per print carry only support.

## Headless build

`headless/` builds the slicing libraries and `slice_stl` (slice one STL with
organic supports and write G-code) from system packages, without the GUI or
the dependency bundle. It exists for sandboxes that cannot download the
dependencies; with a normal build, use the full tree instead.

On Ubuntu 24.04:

    sudo apt install cmake ninja-build g++ libboost-all-dev libtbb-dev libcgal-dev \
        libeigen3-dev libopenvdb-dev libblosc-dev libbgcode-dev libheatshrink-dev \
        libnanosvg-dev libqhull-dev libnlopt-dev libnlopt-cxx-dev libcereal-dev \
        libexpected-dev libspdlog-dev libfmt-dev libmagicenum-dev libjpeg-dev libpng-dev \
        libexpat1-dev libgmp-dev libmpfr-dev
    # PrusaSlicer needs Boost 1.86; build it with the dependency recipe.
    cmake -S deps -B deps/build -G Ninja && LC_ALL=C.UTF-8 ninja -C deps/build dep_Boost
    cmake -S sandboxes/organic_trunk_layers/headless -B build-headless -G Ninja \
        -DCMAKE_PREFIX_PATH=$PWD/deps/build/destdir/usr/local
    ninja -C build-headless slice_stl

`slice_stl part.stl out.gcode [key=value ...]` takes PrusaSlicer config keys,
for example `layer_height=0.2 support_tree_angle=40`.

`headless/stubs` stands in for two libraries the sandbox could not download:
libassert (the ASSERT macros, still checked) and prusa_fdm_mixer (only used for
color previews of mixed filaments). The CMake project also patches copies of two
Qhull 2020.2 headers that C++20 rejects, and adds `stopwatch::elapsed_ms()` for
spdlog older than 1.14.

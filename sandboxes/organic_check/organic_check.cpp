// Slice one STL with organic supports and write the G-code, so the branches can be
// checked with check_floating.py. Uses the fff_print test helpers for a default
// printer / print / filament setup.
//
//     organic_check part.stl part.gcode [layer_height]
//
// PRUSASLICER_EXPORT_ORGANIC_STL and PRUSASLICER_ORGANIC_STATS work as with the slicer.

#include "test_data.hpp"

#include "Slic3r/Biz/Format/STL.hpp"

#include <boost/nowide/fstream.hpp>

#include <cstdio>
#include <cstdlib>

using namespace Slic3r;

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s part.stl part.gcode [layer_height]\n", argv[0]);
        return 2;
    }
    auto mesh = Biz::load_stl(argv[1]);
    if (! mesh) {
        std::fprintf(stderr, "%s: %s\n", argv[1], mesh.error().c_str());
        return 1;
    }

    Test::TestConfig config;
    config.print.items.opt("support_material").set(Domain::SupportMode::Everywhere);
    config.print.items.opt("support_material_style").set(Domain::SupportMaterialStyle::smsOrganic);
    // The test config leaves the XY gap at 0, which makes tree support divide by zero.
    config.print.items.opt("support_material_xy_spacing").set(Domain::FloatOrPercentage{0.3});
    if (argc > 3)
        config.print.items.opt("layer_height").set(std::atof(argv[3]));

    Slic3r::Print print;
    Domain::Model model;
    Test::init_print({ *mesh }, print, model, config);
    std::string gcode = Test::gcode(print);

    boost::nowide::ofstream out(argv[2]);
    out << gcode;
    return out ? 0 : 1;
}

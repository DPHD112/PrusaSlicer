// Slice one STL with organic supports and write the G-code, so the branches can be
// checked with check_floating.py. Uses the fff_print test helpers for a default
// printer / print / filament setup.
//
//     organic_check part.stl part.gcode [layer_height] [key=value ...]
//
// key=value overrides a print setting, for example support_material_threshold=40 or
// support_material_buildplate_only=1. Percentages take a trailing %.
// PRUSASLICER_EXPORT_ORGANIC_STL and PRUSASLICER_ORGANIC_STATS work as with the slicer.

#include "test_data.hpp"

#include "Slic3r/Biz/Format/STL.hpp"

#include <boost/nowide/fstream.hpp>

#include <cstdio>
#include <cstdlib>
#include <string>

using namespace Slic3r;

static bool set_option(Test::TestConfig &config, const std::string &arg)
{
    const size_t eq = arg.find('=');
    if (eq == std::string::npos)
        return false;
    const std::string key   = arg.substr(0, eq);
    const std::string value = arg.substr(eq + 1);
    const bool        percent = ! value.empty() && value.back() == '%';
    Domain::ConfigItem &item = config.print.items.opt(key);
    if (item.holds_alternative<bool>())
        item.set(value == "1" || value == "true");
    else if (item.holds_alternative<int>())
        item.set(std::stoi(value));
    else if (item.holds_alternative<double>())
        item.set(std::stod(value));
    else if (item.holds_alternative<Domain::Percentage>())
        item.set(Domain::Percentage{ std::stod(value) });
    else if (item.holds_alternative<Domain::FloatOrPercentage>())
        item.set(percent ? Domain::FloatOrPercentage{ Domain::Percentage{ std::stod(value) } } : Domain::FloatOrPercentage{ std::stod(value) });
    else
        return false;
    return true;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s part.stl part.gcode [layer_height] [key=value ...]\n", argv[0]);
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
    for (int i = 3; i < argc; ++ i) {
        const std::string arg = argv[i];
        if (arg.find('=') == std::string::npos)
            config.print.items.opt("layer_height").set(std::atof(argv[i]));
        else if (! set_option(config, arg)) {
            std::fprintf(stderr, "%s: can't set this option from the command line\n", argv[i]);
            return 2;
        }
    }

    Slic3r::Print print;
    Domain::Model model;
    Test::init_print({ *mesh }, print, model, config);
    std::string gcode = Test::gcode(print);

    boost::nowide::ofstream out(argv[2]);
    out << gcode;
    return out ? 0 : 1;
}

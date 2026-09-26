// Slice one STL headlessly and write the G-code. Builds against libslic3r only,
// without slic3r-shared or the GUI dependencies.
//
//     slice_stl part.stl part.gcode [key=value ...]
//
// Keys are PrusaSlicer config keys (print, printer, first tool and filament).
// Organic supports everywhere are on by default.

#include "Slic3r/Biz/Algorithms/Model.hpp"
#include "Slic3r/Biz/Algorithms/ModelObject.hpp"
#include "Slic3r/Domain/BedInstance.hpp"
#include "Slic3r/Domain/Bed.hpp"
#include "Slic3r/Domain/ConfigPack.hpp"
#include "Slic3r/Domain/Model.hpp"
#include "Slic3r/Domain/ModelObject.hpp"
#include "Slic3r/Domain/Preset/SelectedPreset.hpp"
#include "Slic3r/Domain/TemplateUtils.hpp"
#include "Slic3r/Domain/TriangleMesh.hpp"
#include "Slic3r/TestUtils/HwConfigUtils.hpp"
#include "libslic3r/Print.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <tuple>

using namespace Slic3r;

static bool load_stl(const char *path, indexed_triangle_set &its)
{
    std::ifstream in(path, std::ios::binary);
    if (! in)
        return false;
    std::string data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::map<std::tuple<float, float, float>, int> index;
    auto vertex = [&](float x, float y, float z) {
        auto [it, inserted] = index.emplace(std::make_tuple(x, y, z), int(its.vertices.size()));
        if (inserted)
            its.vertices.emplace_back(x, y, z);
        return it->second;
    };
    auto add = [&](const float *v) {
        int a = vertex(v[0], v[1], v[2]), b = vertex(v[3], v[4], v[5]), c = vertex(v[6], v[7], v[8]);
        if (a != b && b != c && c != a)
            its.indices.push_back({ a, b, c });
    };
    uint32_t count = 0;
    if (data.size() >= 84)
        std::memcpy(&count, data.data() + 80, 4);
    if (data.size() >= 84 && data.size() == 84 + size_t(count) * 50) {
        for (uint32_t i = 0; i < count; ++ i) {
            float v[9];
            std::memcpy(v, data.data() + 84 + size_t(i) * 50 + 12, sizeof(v));
            add(v);
        }
    } else {
        std::istringstream s(data);
        std::string word;
        float v[9];
        int n = 0;
        while (s >> word)
            if (word == "vertex") {
                s >> v[n] >> v[n + 1] >> v[n + 2];
                if ((n += 3) == 9) {
                    add(v);
                    n = 0;
                }
            }
    }
    return ! its.indices.empty();
}

static Domain::ConfigItem* find_option(Domain::ConfigPackFDM &config, const std::string &key)
{
    for (Domain::ConfigBox *box : { static_cast<Domain::ConfigBox*>(&config.print), static_cast<Domain::ConfigBox*>(&config.printer),
                                    static_cast<Domain::ConfigBox*>(&config.tool.front()), static_cast<Domain::ConfigBox*>(&config.filament.front()) })
        if (Domain::FindResult r = box->find(key); r.item && ! r.is_override)
            return r.item;
    return nullptr;
}

static bool set_option(Domain::ConfigItem &item, const std::string &value)
{
    return item.visit(Domain::overloaded{
        [&](Domain::EnumWrapper &v) { v.set_string(value); return true; },
        [&](bool &v) { v = value == "1" || value == "true"; return true; },
        [&](int &v) { v = std::stoi(value); return true; },
        [&](std::optional<int> &v) { v = std::stoi(value); return true; },
        [&](double &v) { v = std::stod(value); return true; },
        [&](std::string &v) { v = value; return true; },
        [&](Domain::FloatOrPercentage &v) {
            v = value.ends_with('%') ? Domain::FloatOrPercentage(Domain::Percentage{ std::stod(value.substr(0, value.size() - 1)) }) :
                                       Domain::FloatOrPercentage(std::stod(value));
            return true;
        },
        [&](Domain::Percentage &v) { v.value = std::stod(value); return true; },
        [&](auto &) { return false; }
    });
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s part.stl part.gcode [key=value ...]\n", argv[0]);
        return 2;
    }
    indexed_triangle_set its;
    if (! load_stl(argv[1], its)) {
        std::fprintf(stderr, "%s: cannot read the STL\n", argv[1]);
        return 1;
    }

    Domain::ConfigPackFDM config;
    Domain::Preset::HwPrinterConfig hw_config = Test::create_dummy_hw_config();
    // The default XY separation is 50% of the external perimeter width, which is 0 (auto) in this bare config,
    // and a zero separation divides by zero in the tree support.
    std::vector<std::string> settings { "support_material=everywhere", "support_material_style=organic", "support_material_xy_spacing=0.3" };
    for (int i = 3; i < argc; ++ i)
        settings.emplace_back(argv[i]);
    for (const std::string &setting : settings) {
        size_t eq = setting.find('=');
        std::string key = setting.substr(0, eq);
        Domain::ConfigItem *item = eq == std::string::npos ? nullptr : find_option(config, key);
        if (item == nullptr || ! set_option(*item, setting.substr(eq + 1))) {
            std::fprintf(stderr, "cannot set %s\n", setting.c_str());
            return 2;
        }
    }

    Domain::Model model;
    Domain::ModelObject *object = model.add_object();
    object->name = argv[1];
    Biz::Algorithms::ModelObject::add_volume(object, Domain::TriangleMesh(std::move(its)));
    object->add_instance();
    Biz::Algorithms::Model::center_instances_around_point(model, { 100, 100 });
    Biz::Algorithms::ModelObject::ensure_on_bed(*object);

    Domain::Preset::SelectedPresetMetadata preset_metadata{
        .hw_config = hw_config,
        .tools     = std::vector<Domain::Preset::EvaluatedPresetMetadata>{ hw_config.tool_count },
        .materials = std::vector<Domain::Preset::EvaluatedPresetMetadata>{ hw_config.material_slot_count() }
    };
    Domain::Bed bed;
    Domain::BedInstance bed_instance{ bed };
    for (Domain::ModelInstance *instance : object->instances)
        bed_instance.model_instances.push_back(instance);

    auto t0 = std::chrono::steady_clock::now();
    Slic3r::Print print;
    print.update(model, config, bed_instance, preset_metadata,
        [](const Biz::Slicing::IPrint::UniversalPrintStatistics &) { return Biz::Slicing::SerializedConfig{}; });
    print.validate();
    print.process();
    Biz::libpgcode::ProcessorResult result = print.process_gcode();
    auto t1 = std::chrono::steady_clock::now();

    std::ofstream out(argv[2], std::ios::binary);
    out << result.const_gcode()->str();
    std::fprintf(stderr, "sliced %s in %.1f s\n", argv[1], std::chrono::duration<double>(t1 - t0).count());
    return out ? 0 : 1;
}

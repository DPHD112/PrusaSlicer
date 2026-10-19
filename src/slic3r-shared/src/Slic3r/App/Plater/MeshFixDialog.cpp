#include "Slic3r/App/Plater/MeshFixDialog.hpp"

#include "Slic3r/App/Plater/MeshFixColors.hpp"
#include "Slic3r/App/Yoga/AbstractButton.hpp"
#include "Slic3r/App/Yoga/Circle.hpp"
#include "Slic3r/App/Yoga/ComboBox.hpp"
#include "Slic3r/App/Yoga/InputTextWithSpin.hpp"
#include "Slic3r/App/Yoga/LayoutButton.hpp"
#include "Slic3r/App/Yoga/SliderWithInput.hpp"
#include "Slic3r/App/Yoga/Text.hpp"
#include "Slic3r/App/Yoga/ToggleButton.hpp"
#include "Slic3r/App/Yoga/Validator.hpp"
#include "Slic3r/Biz/I18N/I18N.hpp"
#include "Slic3r/LegacyFormat.hpp"

#include <fmt/format.h>
#include <imgui/imgui.h>

using namespace Slic3r::App::Yoga;
using namespace Slic3r::Biz;

namespace Slic3r::App::Plater {

namespace {

ImColor to_imcolor(const Domain::ColorRGBA& color)
{
    return ImColor(color.r(), color.g(), color.b(), color.a());
}

} // namespace

MeshFixDialog::Callbacks& MeshFixDialog::callbacks()
{
    return m_callbacks;
}

MeshFixDialog::MeshFixDialog() : GizmoWindow()
{
    content()->set_orientation(Orientation::Vertical);
    content()->set_gap(gap_size());

    // Mouse tool
    std::unique_ptr<Item> tool_buttons = std::make_unique<Item>();
    tool_buttons->set_gap(5.f);
    tool_buttons->set_flex_wrap(YGWrapWrap);
    m_tool_buttons = {
        add_icon_button(tool_buttons.get(), Render::Icon::Triangle, _u8L("Mark triangle")),
        add_icon_button(
            tool_buttons.get(),
            Render::Icon::Cube,
            _u8L("Mark shell: all triangles connected to the clicked one")
        ),
        add_icon_button(
            tool_buttons.get(),
            Render::Icon::WandMagicSparkles,
            _u8L("Mark surface: stops at edges sharper than the angle tolerance")
        ),
        add_icon_button(
            tool_buttons.get(),
            Render::Icon::Square,
            _u8L("Mark plane: planar region around the clicked triangle")
        ),
        add_icon_button(tool_buttons.get(), Render::Icon::PaintBrush, _u8L("Mark with brush")),
        add_icon_button(
            tool_buttons.get(),
            Render::Icon::FillDrip,
            _u8L("Fill hole: click a hole to fill it")
        )
    };
    add_new_row(_u8L("Tool"), std::move(tool_buttons));
    m_tool_group.set_buttons(
        {m_tool_buttons[0],
         m_tool_buttons[1],
         m_tool_buttons[2],
         m_tool_buttons[3],
         m_tool_buttons[4],
         m_tool_buttons[5]}
    );
    m_tool_group.callbacks().checked_changed =
        [this](AbstractButton* current_checked, AbstractButton*)
    {
        for (size_t i = 0; i < m_tool_buttons.size(); ++i)
            if (current_checked == m_tool_buttons[i])
                m_tool = static_cast<MeshFixTool>(i);
        update_visibility();
        if (!m_ignore_callbacks && m_callbacks.tool_changed)
            m_callbacks.tool_changed(m_tool);
    };

    m_angle_row =
        add_row_with_slider(content(), &m_angle, _u8L("Angle tolerance"), "°")->parent_item();
    m_angle->set_validator(std::make_unique<IntValidator>(0, 90));
    m_angle->set_begin_value(0.);
    m_angle->set_end_value(90.);
    m_angle->set_step(1.);
    m_angle->callbacks().value_changed = [this](double value)
    {
        if (!m_ignore_callbacks && m_callbacks.angle_changed)
            m_callbacks.angle_changed(value);
    };

    m_brush_row =
        add_row_with_slider(content(), &m_brush_radius, _u8L("Brush radius"), "mm")->parent_item();
    m_brush_radius->set_begin_value(0.1);
    m_brush_radius->set_end_value(20.);
    m_brush_radius->set_step(0.1);
    m_brush_radius->set_validator_precision(1);
    m_brush_radius->callbacks().value_changed = [this](double value)
    {
        if (!m_ignore_callbacks && m_callbacks.brush_radius_changed)
            m_callbacks.brush_radius_changed(value);
    };

    // Marking
    Item* mark_buttons = add_buttons_row(content());
    add_action_button(mark_buttons, _u8L("Unmark all"), {}, &Callbacks::clear_marks);
    add_action_button(
        mark_buttons,
        _u8L("Invert"),
        _u8L("Invert the marking"),
        &Callbacks::invert_marks
    );
    add_action_button(
        mark_buttons,
        _u8L("Grow"),
        _u8L("Mark the triangles around the marked ones"),
        &Callbacks::grow_marks
    );
    add_action_button(
        mark_buttons,
        _u8L("Shrink"),
        _u8L("Unmark the triangles on the border of the marking"),
        &Callbacks::shrink_marks
    );

    content()
        ->emplace_back<Text>(_u8L("Mark problems"))
        ->set_font_type(Render::ImguiFontType::Bold);
    Item* problem_buttons = add_buttons_row(content());
    add_action_button(
        problem_buttons,
        _u8L("Bad edges"),
        _u8L("Mark triangles with open or non-manifold edges"),
        &Callbacks::mark_bad_edges
    );
    add_action_button(
        problem_buttons,
        _u8L("Flipped"),
        _u8L("Mark flipped triangles"),
        &Callbacks::mark_flipped
    );
    add_action_button(
        problem_buttons,
        _u8L("Noise shells"),
        _u8L("Mark small separate shells"),
        &Callbacks::mark_noise_shells
    );
    add_action_button(
        problem_buttons,
        _u8L("Intersecting"),
        _u8L("Mark triangles intersecting other triangles"),
        &Callbacks::mark_intersecting
    );

    add_separator(content());

    // Diagnostics
    content()->emplace_back<Text>(_u8L("Diagnostics"))->set_font_type(Render::ImguiFontType::Bold);
    m_triangles_value  = add_stat_row(content(), _u8L("Triangles"), std::nullopt);
    m_marked_value     = add_stat_row(content(), _u8L("Marked"), MeshFixColors::marked());
    m_open_edges_value = add_stat_row(content(), _u8L("Bad edges"), MeshFixColors::open_edge());
    m_holes_value      = add_stat_row(content(), _u8L("Holes"), MeshFixColors::hole());
    m_non_manifold_value =
        add_stat_row(content(), _u8L("Non-manifold edges"), MeshFixColors::non_manifold_edge());
    m_flipped_value = add_stat_row(content(), _u8L("Flipped triangles"), MeshFixColors::flipped());
    m_shells_value  = add_stat_row(content(), _u8L("Shells"), std::nullopt);
    m_noise_shells_value = add_stat_row(content(), _u8L("Noise shells"), MeshFixColors::noise());
    m_intersecting_value =
        add_stat_row(content(), _u8L("Intersecting triangles"), MeshFixColors::intersecting());

    Item* check_buttons = add_buttons_row(content());
    add_action_button(
        check_buttons,
        _u8L("Check intersections"),
        _u8L(
            "Find triangles intersecting other triangles, which may take a while for large meshes"
        ),
        &Callbacks::check_intersections
    );

    add_row_with_toggle_button(_u8L("Highlight problems"), content(), &m_show_problems);
    m_show_problems->set_checked(true);
    m_show_problems->callbacks().checked_changed = [this](bool checked)
    {
        if (!m_ignore_callbacks && m_callbacks.show_problems_changed)
            m_callbacks.show_problems_changed(checked);
    };

    add_separator(content());

    // Edits of the marked triangles
    content()
        ->emplace_back<Text>(_u8L("Marked triangles"))
        ->set_font_type(Render::ImguiFontType::Bold);
    Item* edit_buttons = add_buttons_row(content());
    m_marked_buttons   = {
        add_action_button(
            edit_buttons,
            _u8L("Delete"),
            _u8L("Delete the marked triangles"),
            &Callbacks::delete_marked
        ),
        add_action_button(
            edit_buttons,
            _u8L("Flip"),
            _u8L("Flip the normals of the marked triangles"),
            &Callbacks::flip_marked
        ),
        add_action_button(
            edit_buttons,
            _u8L("Split off"),
            _u8L("Move the marked triangles into a new part"),
            &Callbacks::split_marked
        )
    };

    add_separator(content());

    // Repair
    content()->emplace_back<Text>(_u8L("Repair"))->set_font_type(Render::ImguiFontType::Bold);
    add_row_with_combo_box(_u8L("Hole fill"), content(), &m_fill_mode);
    m_fill_mode->set_items( // Order must match enum class MeshFixFillMode
        {_ctx_u8L("Triangulate", "Mesh fix: hole fill mode"),
         _ctx_u8L("Flat", "Mesh fix: hole fill mode"),
         _ctx_u8L("Smooth", "Mesh fix: hole fill mode")}
    );
    m_fill_mode->callbacks().selection_changed = [this](int index)
    {
        if (!m_ignore_callbacks && m_callbacks.fill_mode_changed)
            m_callbacks.fill_mode_changed(static_cast<MeshFixFillMode>(index));
    };

    add_row_with_spin_double(
        _u8L("Stitch tolerance"),
        content(),
        &m_stitch_tolerance,
        "mm",
        std::string{},
        0.,
        1.,
        0.001,
        0.01
    );
    m_stitch_tolerance->callbacks().text_edited = [this]()
    {
        const auto* validator =
            dynamic_cast<const DoubleValidator*>(m_stitch_tolerance->validator());
        if (!m_ignore_callbacks && validator != nullptr && m_callbacks.stitch_tolerance_changed)
            m_callbacks.stitch_tolerance_changed(validator->value());
    };

    Item* repair_buttons = add_buttons_row(content());
    add_action_button(
        repair_buttons,
        _u8L("Fill holes"),
        _u8L("Fill all holes"),
        &Callbacks::fill_all_holes
    );
    add_action_button(
        repair_buttons,
        _u8L("Unify normals"),
        _u8L("Flip the triangles facing inwards"),
        &Callbacks::unify_normals
    );
    add_action_button(
        repair_buttons,
        _u8L("Stitch"),
        _u8L("Merge open edge vertices closer than the stitch tolerance"),
        &Callbacks::stitch
    );
    add_action_button(
        repair_buttons,
        _u8L("Remove noise shells"),
        {},
        &Callbacks::remove_noise_shells
    );

    add_separator(content());

    m_help_factory.init(add_non_shrinked_wrap(content(), Orientation::Vertical, gap_size()));
    m_help_factory.add_item(
        {GizmoHelpFactory::HelpIcon{Render::Icon::MouseLeft}},
        _u8L("Mark, or fill a hole")
    );
    m_help_factory
        .add_item({{"SHIFT"}, GizmoHelpFactory::HelpIcon{Render::Icon::MouseLeft}}, _u8L("Unmark"));
    m_help_factory.add_item(
        {{"ALT"}, GizmoHelpFactory::HelpIcon{Render::Icon::MouseWheel}},
        _u8L("Brush radius or angle tolerance")
    );
    m_help_factory.add_item({{"DEL"}}, _u8L("Delete marked triangles"));

    // One click repair
    bottom_bar()->set_flex_shrink(0.f);
    bottom_bar()->set_padding(2.f * gap_size());
    bottom_bar()->set_gap(gap_size());
    LayoutButton* auto_fix = bottom_bar()->emplace_back<LayoutButton>(
        _u8L("Auto fix"),
        Render::Icon::None,
        _u8L(
            "Remove degenerate and duplicate triangles, stitch cracks, unify normals, remove noise shells and fill holes"
        )
    );
    auto_fix->set_flex_grow(1.f);
    auto_fix->set_background_color(m_theme->color_imgui(Platform::Color::AccentPrimary));
    auto_fix->set_label_font_type(Render::ImguiFontType::Bold);
    auto_fix->set_content_padding(gap_size());
    auto_fix->callbacks().action = [this]()
    {
        if (m_callbacks.auto_fix)
            m_callbacks.auto_fix();
    };

    set_stats(m_stats);
    update_visibility();
}

Item* MeshFixDialog::add_buttons_row(Item* parent)
{
    Item* row = add_non_shrinked_wrap(parent, Orientation::Horizontal, gap_size());
    row->set_flex_wrap(YGWrapWrap);
    return row;
}

LayoutButton* MeshFixDialog::add_action_button(
    Item* parent,
    const std::string& label,
    const std::string& tooltip,
    std::function<void()> Callbacks::* callback
)
{
    LayoutButton* button = parent->emplace_back<LayoutButton>(label, Render::Icon::None, tooltip);
    button->set_content_padding({10.f, 5.f});
    button->set_background_color(m_theme->color_imgui(Platform::Color::Button));
    button->callbacks().action = [this, callback]()
    {
        if (m_callbacks.*callback)
            (m_callbacks.*callback)();
    };
    return button;
}

Text* MeshFixDialog::add_stat_row(
    Item* parent,
    const std::string& title,
    const std::optional<Domain::ColorRGBA>& color
)
{
    Item* row = parent->emplace_back<Item>();
    row->set_gap(gap_size());
    row->set_flex_shrink(0.f);
    row->set_align_items(YGAlignCenter);
    Circle* marker = row->emplace_back<Circle>();
    marker->set_height(12_fpx);
    marker->set_flex_shrink(0.f);
    // Rows without a color keep the marker space to stay aligned.
    marker->set_fill(color.has_value() ? to_imcolor(*color) : ImColor(0.f, 0.f, 0.f, 0.f));
    row->emplace_back<Text>(title)->set_flex_grow(1.f);
    Text* value = row->emplace_back<Text>("0");
    value->set_font_type(Render::ImguiFontType::Bold);
    return value;
}

void MeshFixDialog::set_tool(MeshFixTool tool)
{
    m_ignore_callbacks = true;
    m_tool_buttons[static_cast<size_t>(tool)]->set_checked(true);
    m_ignore_callbacks = false;
    m_tool             = tool;
    update_visibility();
}

void MeshFixDialog::set_angle(double angle_deg)
{
    m_ignore_callbacks = true;
    m_angle->set_value(angle_deg);
    m_ignore_callbacks = false;
}

void MeshFixDialog::set_brush_radius(double radius)
{
    m_ignore_callbacks = true;
    m_brush_radius->set_value(radius);
    m_ignore_callbacks = false;
}

void MeshFixDialog::set_fill_mode(MeshFixFillMode mode)
{
    m_fill_mode->set_current_index(static_cast<int>(mode));
}

void MeshFixDialog::set_stitch_tolerance(double tolerance)
{
    m_stitch_tolerance->set_text(fmt::format("{:.3f}", tolerance));
}

void MeshFixDialog::set_show_problems(bool show)
{
    m_ignore_callbacks = true;
    m_show_problems->set_checked(show);
    m_ignore_callbacks = false;
}

void MeshFixDialog::set_stats(const MeshFixStats& stats)
{
    m_stats = stats;
    m_triangles_value->set_text(std::to_string(stats.triangles));
    m_marked_value->set_text(std::to_string(stats.marked));
    m_open_edges_value->set_text(std::to_string(stats.open_edges));
    m_holes_value->set_text(std::to_string(stats.holes));
    m_non_manifold_value->set_text(std::to_string(stats.non_manifold_edges));
    m_flipped_value->set_text(std::to_string(stats.flipped));
    m_shells_value->set_text(std::to_string(stats.shells));
    m_noise_shells_value->set_text(std::to_string(stats.noise_shells));
    // TRN Mesh fix: The intersections were not checked yet.
    m_intersecting_value->set_text(
        stats.intersecting.has_value() ? std::to_string(*stats.intersecting) : _u8L("not checked")
    );
    for (LayoutButton* button : m_marked_buttons)
        button->set_enabled(stats.marked > 0);
    update_warning();
}

void MeshFixDialog::set_painting_warning(bool show)
{
    m_painting_warning = show;
    update_warning();
}

void MeshFixDialog::update_visibility()
{
    m_angle_row->set_visible(
        m_tool == MeshFixTool::MarkSurface || m_tool == MeshFixTool::MarkPlane
    );
    m_brush_row->set_visible(m_tool == MeshFixTool::MarkBrush);
}

void MeshFixDialog::update_warning()
{
    std::vector<std::string> problems;
    if (m_stats.holes > 0)
        problems.emplace_back(Slic3r::format(_u8L("Holes: %1%"), m_stats.holes));
    if (m_stats.open_edges > 0)
        problems.emplace_back(Slic3r::format(_u8L("Bad edges: %1%"), m_stats.open_edges));
    if (m_stats.non_manifold_edges > 0)
        problems.emplace_back(
            Slic3r::format(_u8L("Non-manifold edges: %1%"), m_stats.non_manifold_edges)
        );
    if (m_stats.flipped > 0)
        problems.emplace_back(Slic3r::format(_u8L("Flipped triangles: %1%"), m_stats.flipped));
    if (m_stats.noise_shells > 0)
        problems.emplace_back(Slic3r::format(_u8L("Noise shells: %1%"), m_stats.noise_shells));
    if (m_stats.intersecting.value_or(0) > 0)
        problems.emplace_back(
            Slic3r::format(_u8L("Intersecting triangles: %1%"), *m_stats.intersecting)
        );
    if (m_painting_warning)
        problems.emplace_back(_u8L(
            "Editing the mesh removes the painted supports, seams, fuzzy skin and colors of this part."
        ));

    if (problems.empty())
        clear_warning();
    else
        set_warning(_u8L("Mesh problems"), problems);
}

} // namespace Slic3r::App::Plater

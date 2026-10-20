#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "Slic3r/App/Plater/GizmoWindow.hpp"
#include "Slic3r/App/Yoga/ButtonGroup.hpp"
#include "Slic3r/Domain/Color.hpp"

namespace Slic3r::App::Yoga {
class ComboBox;
class InputTextWithSpin;
class LayoutButton;
class SliderWithInput;
class Text;
class ToggleButton;
} // namespace Slic3r::App::Yoga

namespace Slic3r::App::Plater {

/**
 * @brief What a click in the 3D view does, the order matches the tool buttons.
 */
enum class MeshFixTool
{
    MarkTriangle,
    MarkShell,
    MarkSurface,
    MarkPlane,
    MarkBrush,
    FillHole
};

/**
 * @brief How holes are filled, the order matches the combo box items.
 */
enum class MeshFixFillMode
{
    Triangulate,
    Flat,
    Smooth
};

struct MeshFixStats
{
    size_t triangles{0};
    size_t marked{0};
    size_t open_edges{0};
    size_t non_manifold_edges{0};
    size_t holes{0};
    size_t flipped{0};
    size_t shells{0};
    size_t noise_shells{0};
    // Empty until the intersections were checked.
    std::optional<size_t> intersecting;
    // Triangles thinner than the minimum wall, empty until the wall thickness was analyzed.
    std::optional<size_t> thin_faces;
    // Thinnest wall in mm, infinity if none was measured.
    float thinnest_wall{std::numeric_limits<float>::infinity()};
};

/**
 * @brief Sidebar panel of the mesh fix tool.
 */
class MeshFixDialog : public GizmoWindow
{
public:
    MeshFixDialog();

    struct Callbacks
    {
        std::function<void(MeshFixTool)> tool_changed{nullptr};
        std::function<void(double)> angle_changed{nullptr};
        std::function<void(double)> brush_radius_changed{nullptr};
        std::function<void(MeshFixFillMode)> fill_mode_changed{nullptr};
        std::function<void(double)> stitch_tolerance_changed{nullptr};
        std::function<void(bool)> show_problems_changed{nullptr};
        std::function<void(double)> min_wall_changed{nullptr};
        std::function<void(bool)> show_thickness_changed{nullptr};

        std::function<void()> clear_marks{nullptr};
        std::function<void()> invert_marks{nullptr};
        std::function<void()> grow_marks{nullptr};
        std::function<void()> shrink_marks{nullptr};
        std::function<void()> mark_bad_edges{nullptr};
        std::function<void()> mark_flipped{nullptr};
        std::function<void()> mark_noise_shells{nullptr};
        std::function<void()> mark_intersecting{nullptr};

        std::function<void()> delete_marked{nullptr};
        std::function<void()> flip_marked{nullptr};
        std::function<void()> split_marked{nullptr};

        std::function<void()> fill_all_holes{nullptr};
        std::function<void()> unify_normals{nullptr};
        std::function<void()> stitch{nullptr};
        std::function<void()> remove_noise_shells{nullptr};
        std::function<void()> check_intersections{nullptr};
        std::function<void()> analyze_thickness{nullptr};
        std::function<void()> mark_thin_walls{nullptr};
        std::function<void()> auto_fix{nullptr};
    };

    Callbacks& callbacks();

    /**
     * @name Setters of the shown state, they do not invoke the callbacks.
     * @{
     */
    void set_tool(MeshFixTool tool);
    void set_angle(double angle_deg);
    void set_brush_radius(double radius);
    void set_fill_mode(MeshFixFillMode mode);
    void set_stitch_tolerance(double tolerance);
    void set_show_problems(bool show);
    void set_min_wall(double thickness);
    void set_show_thickness(bool show);
    void set_stats(const MeshFixStats& stats);
    /**
     * @brief Wall thickness under the cursor in mm, empty if there is none to show.
     */
    void set_cursor_thickness(std::optional<float> thickness);
    /**@}*/

    /**
     * @brief Warn that applying an edit loses the painting of the part.
     */
    void set_painting_warning(bool show);
    /**
     * @brief Explain why the last edit was refused, an empty text hides it.
     */
    void set_notice(const std::string& text);

private:
    Yoga::Text* add_stat_row(
        Yoga::Item* parent,
        const std::string& title,
        const std::optional<Domain::ColorRGBA>& color
    );
    Yoga::LayoutButton* add_action_button(
        Yoga::Item* parent,
        const std::string& label,
        const std::string& tooltip,
        std::function<void()> Callbacks::* callback
    );
    Yoga::Item* add_buttons_row(Yoga::Item* parent);
    void update_visibility();
    void update_warning();

    Callbacks m_callbacks;
    // Set while the gizmo pushes its state into the widgets, which would invoke the callbacks.
    bool m_ignore_callbacks{false};

    MeshFixTool m_tool{MeshFixTool::MarkTriangle};
    Yoga::ButtonGroup m_tool_group;
    std::array<Yoga::LayoutButton*, 6> m_tool_buttons{};

    Yoga::Item* m_angle_row{nullptr};
    Yoga::SliderWithInput* m_angle{nullptr};
    Yoga::Item* m_brush_row{nullptr};
    Yoga::SliderWithInput* m_brush_radius{nullptr};
    Yoga::ComboBox* m_fill_mode{nullptr};
    Yoga::InputTextWithSpin* m_stitch_tolerance{nullptr};
    Yoga::ToggleButton* m_show_problems{nullptr};
    Yoga::InputTextWithSpin* m_min_wall{nullptr};
    Yoga::ToggleButton* m_show_thickness{nullptr};
    double m_min_wall_value{1.};

    Yoga::Text* m_triangles_value{nullptr};
    Yoga::Text* m_marked_value{nullptr};
    Yoga::Text* m_open_edges_value{nullptr};
    Yoga::Text* m_holes_value{nullptr};
    Yoga::Text* m_non_manifold_value{nullptr};
    Yoga::Text* m_flipped_value{nullptr};
    Yoga::Text* m_shells_value{nullptr};
    Yoga::Text* m_noise_shells_value{nullptr};
    Yoga::Text* m_intersecting_value{nullptr};
    Yoga::Text* m_thin_value{nullptr};
    Yoga::Text* m_thinnest_value{nullptr};
    Yoga::Text* m_cursor_thickness_value{nullptr};

    // Enabled only when some triangles are marked.
    std::vector<Yoga::LayoutButton*> m_marked_buttons;

    MeshFixStats m_stats;
    bool m_painting_warning{false};
    std::string m_notice;
};

} // namespace Slic3r::App::Plater

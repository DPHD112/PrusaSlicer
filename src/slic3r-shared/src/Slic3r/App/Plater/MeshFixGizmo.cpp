#include "Slic3r/App/Plater/MeshFixGizmo.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <magic_enum/magic_enum_flags.hpp>

#include "Slic3r/App/Plater/MeshFixColors.hpp"
#include "Slic3r/App/Plater/MeshFixDialog.hpp"
#include "Slic3r/App/Plater/MeshFixRenderer.hpp"
#include "Slic3r/App/Plater/PlaterScenePresenter.hpp"
#include "Slic3r/App/Platform/ICommand.hpp"
#include "Slic3r/App/Platform/KeyCode.hpp"
#include "Slic3r/App/Platform/KeyModifers.hpp"
#include "Slic3r/App/Platform/KeyboardShortcut.hpp"
#include "Slic3r/App/Platform/MouseEvent.hpp"
#include "Slic3r/App/Scene/Camera.hpp"
#include "Slic3r/App/Scene/SceneNodeTag.hpp"
#include "Slic3r/App/Scene/TriangleMeshManager.hpp"
#include "Slic3r/Assert.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/Session.hpp"
#include "Slic3r/Biz/Algorithms/ModelObject.hpp"
#include "Slic3r/Biz/Algorithms/TriangleMesh.hpp"
#include "Slic3r/Biz/I18N/I18N.hpp"
#include "Slic3r/Biz/Scene/Selection.hpp"
#include "Slic3r/Biz/Scene/SelectionExtents.hpp"
#include "Slic3r/Biz/Utils/MeshRaycaster.hpp"
#include "Slic3r/Domain/Model.hpp"
#include "Slic3r/Domain/ModelInstance.hpp"
#include "Slic3r/Domain/ModelObject.hpp"
#include "Slic3r/Domain/ModelVolume.hpp"
#include "Slic3r/Domain/Project.hpp"

using namespace magic_enum::bitwise_operators;

using Slic3r::Domain::Transform3d;
using Slic3r::Domain::Vec2d;
using Slic3r::Domain::Vec3d;
using Slic3r::Domain::Vec3f;

namespace MeshFix = Slic3r::Biz::Algorithms::MeshFix;

namespace Slic3r::App::Plater {

namespace {

constexpr double BrushRadiusMin  = 0.1;
constexpr double BrushRadiusMax  = 20.;
constexpr double BrushRadiusStep = 0.2;
constexpr double AngleMax        = 90.;
// Maximal distance of triangles marked by the plane tool from the clicked plane.
constexpr double PlaneDistance = 0.01;

std::optional<MeshFix::MarkTool> to_mark_tool(MeshFixTool tool)
{
    switch (tool) {
    case MeshFixTool::MarkTriangle:
        return MeshFix::MarkTool::Triangle;
    case MeshFixTool::MarkShell:
        return MeshFix::MarkTool::Shell;
    case MeshFixTool::MarkSurface:
        return MeshFix::MarkTool::Surface;
    case MeshFixTool::MarkPlane:
        return MeshFix::MarkTool::Plane;
    case MeshFixTool::MarkBrush:
        return MeshFix::MarkTool::Brush;
    case MeshFixTool::FillHole:
        break;
    }
    return std::nullopt;
}

MeshFix::HoleFillMode to_fill_mode(MeshFixFillMode mode)
{
    switch (mode) {
    case MeshFixFillMode::Triangulate:
        return MeshFix::HoleFillMode::Triangulate;
    case MeshFixFillMode::Flat:
        return MeshFix::HoleFillMode::Flat;
    case MeshFixFillMode::Smooth:
        break;
    }
    return MeshFix::HoleFillMode::Smooth;
}

} // namespace

MeshFixGizmo::MeshFixGizmo(
    Render::Device& device,
    Scene::GeometryDataFactory& data_factory,
    Biz::ProjectInteractor& project_interactor,
    PlaterScenePresenter& scene_presenter
) :
    m_project_interactor(project_interactor),
    m_scene_presenter(scene_presenter),
    m_renderer(std::make_unique<MeshFixRenderer>(device, data_factory))
{
    m_dialog = std::make_unique<MeshFixDialog>();
    push_dialog_state();

    MeshFixDialog::Callbacks& callbacks = m_dialog->callbacks();
    callbacks.tool_changed              = [this](MeshFixTool tool) { set_tool(tool); };
    callbacks.angle_changed             = [this](double angle)
    {
        if (m_tool == MeshFixTool::MarkPlane)
            m_plane_angle_deg = angle;
        else
            m_surface_angle_deg = angle;
        m_preview.clear();
        update_preview();
    };
    callbacks.brush_radius_changed = [this](double radius)
    {
        m_brush_radius = radius;
        refresh_cursor();
    };
    callbacks.fill_mode_changed        = [this](MeshFixFillMode mode) { m_fill_mode = mode; };
    callbacks.stitch_tolerance_changed = [this](double tolerance)
    { m_stitch_tolerance = tolerance; };
    callbacks.show_problems_changed = [this](bool show)
    {
        m_show_problems = show;
        refresh_marking();
        refresh_mesh_edges();
    };
    callbacks.min_wall_changed = [this](double thickness)
    {
        m_min_wall = thickness;
        refresh_marking();
        refresh_dialog();
    };
    callbacks.show_thickness_changed = [this](bool show)
    {
        m_show_thickness = show;
        refresh_marking();
    };

    // Marking changes stay in the tool, they are not undoable.
    const auto marking = [this](void (MeshFix::Session::*operation)())
    {
        return [this, operation]()
        {
            if (!m_session)
                return;
            ((*m_session).*operation)();
            refresh_marking();
            refresh_dialog();
        };
    };
    callbacks.clear_marks       = marking(&MeshFix::Session::clear_marks);
    callbacks.invert_marks      = marking(&MeshFix::Session::invert_marks);
    callbacks.grow_marks        = marking(&MeshFix::Session::grow_marks);
    callbacks.shrink_marks      = marking(&MeshFix::Session::shrink_marks);
    callbacks.mark_bad_edges    = marking(&MeshFix::Session::mark_bad_edge_faces);
    callbacks.mark_flipped      = marking(&MeshFix::Session::mark_flipped_faces);
    callbacks.mark_noise_shells = marking(&MeshFix::Session::mark_noise_shells);
    callbacks.mark_intersecting = marking(&MeshFix::Session::mark_intersecting_faces);

    callbacks.delete_marked       = [this]() { delete_marked(); };
    callbacks.flip_marked         = [this]() { flip_marked(); };
    callbacks.split_marked        = [this]() { split_marked(); };
    callbacks.fill_all_holes      = [this]() { fill_all_holes(); };
    callbacks.unify_normals       = [this]() { unify_normals(); };
    callbacks.stitch              = [this]() { stitch(); };
    callbacks.remove_noise_shells = [this]() { remove_noise_shells(); };
    callbacks.check_intersections = [this]() { check_intersections(); };
    callbacks.analyze_thickness   = [this]() { analyze_thickness(); };
    callbacks.mark_thin_walls     = [this]() { mark_thin_walls(); };
    callbacks.auto_fix            = [this]() { auto_fix(); };
}

MeshFixGizmo::~MeshFixGizmo() = default;

Scene::ToolType MeshFixGizmo::type() const
{
    return Scene::ToolType::MeshFixGizmo;
}

bool MeshFixGizmo::disable_object_selection() const
{
    return true;
}

std::unique_ptr<GizmoWindow> MeshFixGizmo::release_ui_window()
{
    return m_dialog.release();
}

void MeshFixGizmo::provide_gizmo_controller(Scene::IGizmoController& gizmo_controller)
{
    m_gizmo_controller = &gizmo_controller;
}

std::optional<Domain::ElementRef> MeshFixGizmo::find_target() const
{
    const Biz::Scene::SceneInteractor& scene_interactor = m_project_interactor.scene_interactor();
    const Biz::Scene::ObjectSelection& selection        = scene_interactor.object_selection();
    if (selection.empty() || selection.contains_wipe_tower() || !selection.only_single_object())
        return std::nullopt;

    const Domain::Project& project = m_project_interactor.selected_project();
    std::optional<Domain::ElementRef> target;
    for (const Domain::ElementRef& ref : scene_interactor.selected_volumes()) {
        const Domain::ModelVolume* volume = project.find_volume_by_id(ref.object_id, ref.volume_id);
        if (volume == nullptr || !volume->is_model_part())
            continue;
        // A single part only, select a part of a multipart object to fix it.
        if (target.has_value())
            return std::nullopt;
        target = ref;
    }
    if (!target.has_value())
        return std::nullopt;

    const Domain::ModelObject* object = project.find_object_by_id(target->object_id);
    const Domain::ModelVolume* volume =
        project.find_volume_by_id(target->object_id, target->volume_id);
    // Text and SVG volumes would be regenerated by their tools, losing the fixes.
    // The parts of a cut object have to stay matching the other parts.
    if (object == nullptr
        || object->is_cut()
        || volume->is_text()
        || volume->is_svg()
        || volume->mesh().empty())
        return std::nullopt;
    return target;
}

bool MeshFixGizmo::enabled() const
{
    return find_target().has_value();
}

Scene::Scene& MeshFixGizmo::scene()
{
    return m_scene_presenter.project_scene(m_project_id);
}

void MeshFixGizmo::on_activated()
{
    m_target = find_target();
    if (!m_target.has_value()) {
        if (m_gizmo_controller != nullptr)
            m_gizmo_controller->deactivate_current_tool();
        return;
    }
    activate();
}

void MeshFixGizmo::on_deactivated()
{
    release();
}

void MeshFixGizmo::on_project_activated(size_t)
{
    on_activated();
}

void MeshFixGizmo::on_project_deactivated(size_t)
{
    release();
}

void MeshFixGizmo::activate()
{
    m_project_id = m_project_interactor.selected_project_id();
    m_active     = true;
    m_renderer->create(scene(), Transform3d::Identity());
    load_target();
    add_listeners();
}

void MeshFixGizmo::release()
{
    if (!m_active)
        return;
    remove_listeners();
    restore_hidden_nodes();
    m_renderer->destroy(scene());
    m_session.reset();
    m_pick_mesh.reset();
    m_target.reset();
    m_hover = {};
    m_preview.clear();
    m_dragging = false;
    m_active   = false;
}

void MeshFixGizmo::add_listeners()
{
    Biz::Scene::SceneInteractor& scene_interactor = m_project_interactor.scene_interactor();
    scene_interactor.add_listener<Biz::Scene::ISceneChangedListener>(this);
    scene_interactor.add_listener<Biz::Scene::ISceneSelectionChangedListener>(this);
    scene().add_listener<App::Scene::ISceneChangedListener>(this);
    scene().add_listener<Scene::IThumbnailRenderListener>(this);
}

void MeshFixGizmo::remove_listeners()
{
    Biz::Scene::SceneInteractor& scene_interactor = m_project_interactor.scene_interactor();
    scene_interactor.remove_listener<Biz::Scene::ISceneChangedListener>(this);
    scene_interactor.remove_listener<Biz::Scene::ISceneSelectionChangedListener>(this);
    scene().remove_listener<App::Scene::ISceneChangedListener>(this);
    scene().remove_listener<Scene::IThumbnailRenderListener>(this);
}

void MeshFixGizmo::load_target()
{
    const Domain::Project& project = m_project_interactor.selected_project();
    const Domain::ModelVolume* volume =
        project.find_volume_by_id(m_target->object_id, m_target->volume_id);
    ASSERT(volume != nullptr);
    m_session = std::make_unique<MeshFix::Session>(volume->mesh().its);
    m_dialog->set_painting_warning(volume->is_painted());
    m_dragging = false;
    update_world_transform();
    hide_target_nodes();
    refresh_mesh();
}

void MeshFixGizmo::update_world_transform()
{
    const Domain::Project& project = m_project_interactor.selected_project();
    const Domain::ModelInstance* instance =
        project.find_instance_by_id(m_target->object_id, m_target->instance_id);
    const Domain::ModelVolume* volume =
        project.find_volume_by_id(m_target->object_id, m_target->volume_id);
    if (instance == nullptr || volume == nullptr)
        return;
    const Transform3d previous = m_world_trafo;
    m_world_trafo              = instance->get_matrix() * volume->get_matrix();
    m_world_scale              = std::cbrt(std::abs(m_world_trafo.linear().determinant()));
    if (!(m_world_scale > 0.))
        m_world_scale = 1.;
    m_renderer->set_transform(m_world_trafo);

    // Moving or rotating the part keeps its wall thickness, scaling does not.
    const auto metric = [](const Transform3d& trafo) -> Eigen::Matrix3d
    { return trafo.linear().transpose() * trafo.linear(); };
    if (m_session
        && m_session->thickness_analyzed()
        && !metric(m_world_trafo).isApprox(metric(previous)))
    {
        m_session->clear_thickness();
        refresh_marking();
        refresh_dialog();
        refresh_cursor_thickness();
    }
}

void MeshFixGizmo::hide_target_nodes()
{
    // The presenter keeps a node for each instance of the volume, only the edited instance is
    // replaced by the tool. The other instances show the model, which is updated by every edit.
    Scene::Node::NodeList nodes;
    const Domain::ElementRef target = *m_target;
    scene().root().query(
        [&target](const Scene::Node* node) -> bool
        {
            const Scene::SceneNodeTag* tag = node->tag_of_type<Scene::SceneNodeTag>();
            return tag != nullptr && tag->matches_element(target);
        },
        nodes,
        true
    );
    for (Scene::Node* node : nodes) {
        node->set_enabled(false);
        if (std::find(m_hidden_node_ids.begin(), m_hidden_node_ids.end(), node->id())
            == m_hidden_node_ids.end())
            m_hidden_node_ids.push_back(node->id());
    }
}

void MeshFixGizmo::restore_hidden_nodes()
{
    for (size_t node_id : m_hidden_node_ids)
        if (Scene::Node* node = scene().node(node_id); node != nullptr)
            node->set_enabled(true);
    m_hidden_node_ids.clear();
}

void MeshFixGizmo::on_node_added(App::Scene::Node* node)
{
    if (!m_target.has_value())
        return;
    const Scene::SceneNodeTag* tag = node->tag_of_type<Scene::SceneNodeTag>();
    if (tag != nullptr && tag->matches_element(*m_target)) {
        node->set_enabled(false);
        m_hidden_node_ids.push_back(node->id());
    }
}

void MeshFixGizmo::on_node_removed(App::Scene::Node* node)
{
    std::erase(m_hidden_node_ids, node->id());
}

void MeshFixGizmo::on_thumbnail_render_begin()
{
    m_renderer->set_enabled(false);
    for (size_t node_id : m_hidden_node_ids)
        if (Scene::Node* node = scene().node(node_id); node != nullptr)
            node->set_enabled(true);
}

void MeshFixGizmo::on_thumbnail_render_end()
{
    m_renderer->set_enabled(true);
    for (size_t node_id : m_hidden_node_ids)
        if (Scene::Node* node = scene().node(node_id); node != nullptr)
            node->set_enabled(false);
}

void MeshFixGizmo::on_model_reloaded(Domain::SelectionId project_id)
{
    // Undo or redo replaced the model, the volume may have an older mesh now.
    if (!m_active || m_committing || project_id != m_project_id)
        return;
    const std::optional<Domain::ElementRef> target = find_target();
    if (!target.has_value()) {
        // The tool gets deactivated by the gizmo manager once it finds the tool disabled.
        return;
    }
    restore_hidden_nodes();
    m_target = target;
    load_target();
}

void MeshFixGizmo::
    on_volume_mesh_changed(Domain::SelectionId project_id, const Domain::ElementRefs& volumes)
{
    if (!m_active || m_committing || project_id != m_project_id)
        return;
    const bool target_changed = std::any_of(
        volumes.begin(),
        volumes.end(),
        [this](const Domain::ElementRef& ref) { return ref.object_id == m_target->object_id; }
    );
    if (target_changed)
        on_model_reloaded(project_id);
}

void MeshFixGizmo::on_volume_transformed(
    Domain::SelectionId project_id,
    const Domain::ElementRefs&,
    Biz::Scene::TransformState,
    const Biz::BedTrackingChanges&
)
{
    if (m_active && !m_committing && project_id == m_project_id)
        update_world_transform();
}

void MeshFixGizmo::on_instance_transformed(
    Domain::SelectionId project_id,
    const Domain::ElementRefs&,
    Biz::Scene::TransformState,
    const Biz::BedTrackingChanges&
)
{
    if (m_active && !m_committing && project_id == m_project_id)
        update_world_transform();
}

void MeshFixGizmo::
    on_scene_selection_changed(Domain::SelectionId project_id, const Biz::Scene::ObjectSelection&)
{
    if (!m_active || m_committing || project_id != m_project_id)
        return;
    const std::optional<Domain::ElementRef> target = find_target();
    if (!target.has_value() || *target == *m_target)
        return;
    // Another part was selected, for example in the object list.
    restore_hidden_nodes();
    m_target = target;
    load_target();
}

void MeshFixGizmo::register_commands(Platform::CommandRegistry& registry)
{
    registry.register_command(
        std::make_unique<Platform::FuncCommand>(
            "mesh-fix-gizmo-delete-marked",
            [this]() { delete_marked(); },
            Platform::FuncCommandExtraOpts{
                .keyboard_shortcuts = Platform::KeyboardShortcuts{
                    Platform::KeyboardShortcut{0, Platform::KeyCode::Delete},
                    Platform::KeyboardShortcut{0, Platform::KeyCode::Backspace}
                }
            }
        )
    );
}

Scene::GizmoActivationState MeshFixGizmo::on_mouse(Scene::GizmoEventContext& ctx, bool)
{
    using Platform::KeyModifier;
    using Platform::KeyModifiers;
    using Platform::MouseButton;
    using Platform::MouseEvent;
    using Scene::GizmoActivationState;

    if (!m_active || !m_session)
        return GizmoActivationState::Inactive;

    const MouseEvent& mouse_event = ctx.mouse_event();
    const bool left_button        = (mouse_event.button() & MouseButton::Left) == MouseButton::Left;
    const bool alt_down   = (mouse_event.key_modifiers() & KeyModifiers(KeyModifier::Alt)) != 0;
    const bool ctrl_down  = (mouse_event.key_modifiers() & KeyModifiers(KeyModifier::Ctrl)) != 0;
    const bool shift_down = (mouse_event.key_modifiers() & KeyModifiers(KeyModifier::Shift)) != 0;

    if (mouse_event.type() == MouseEvent::Type::Wheel) {
        if (!alt_down || mouse_event.wheel_delta_y() == 0.f)
            return GizmoActivationState::Inactive;
        const double direction = mouse_event.wheel_delta_y() > 0.f ? 1. : -1.;
        if (m_tool == MeshFixTool::MarkBrush) {
            m_brush_radius = std::
                clamp(m_brush_radius + direction * BrushRadiusStep, BrushRadiusMin, BrushRadiusMax);
            m_dialog->set_brush_radius(m_brush_radius);
            refresh_cursor();
            return GizmoActivationState::Done;
        }
        if (m_tool == MeshFixTool::MarkSurface || m_tool == MeshFixTool::MarkPlane) {
            double& angle =
                m_tool == MeshFixTool::MarkPlane ? m_plane_angle_deg : m_surface_angle_deg;
            angle = std::clamp(angle + direction, 0., AngleMax);
            m_dialog->set_angle(angle);
            m_preview.clear();
            update_preview();
            return GizmoActivationState::Done;
        }
        return GizmoActivationState::Inactive;
    }

    update_hover(ctx);

    // Mouse moves outside of the view are not reported, a brush stroke continues from where the
    // mouse comes back.
    if (m_dragging && mouse_event.type() == MouseEvent::Type::Enter) {
        m_last_brush_position = Vec2d{ctx.screen_mouse_x(), ctx.screen_mouse_y()};
        m_last_brush_point.reset();
    }

    if (mouse_event.type() == MouseEvent::Type::Move && !m_dragging)
        return GizmoActivationState::Inactive;

    // The second press of a quick double click comes as a double click event.
    const bool button_down = mouse_event.type() == MouseEvent::Type::ButtonDown
        || mouse_event.type() == MouseEvent::Type::DoubleClick;
    if (left_button && button_down && !ctrl_down) {
        if (m_tool == MeshFixTool::FillHole) {
            if (m_hover.hole < 0)
                return GizmoActivationState::Inactive;
            m_dragging = true;
            fill_hovered_hole();
            return GizmoActivationState::Active;
        }
        if (m_hover.face < 0)
            return GizmoActivationState::Inactive;
        m_dragging            = true;
        m_drag_adds           = !shift_down;
        m_last_brush_position = Vec2d{ctx.screen_mouse_x(), ctx.screen_mouse_y()};
        m_last_brush_point    = m_world_trafo * m_hover.point.cast<double>();
        apply_marking_at_hover(m_drag_adds);
        return GizmoActivationState::Active;
    }

    if (mouse_event.type() == MouseEvent::Type::Move && m_dragging) {
        if (m_tool == MeshFixTool::MarkBrush)
            brush_stroke_to(Vec2d{ctx.screen_mouse_x(), ctx.screen_mouse_y()});
        return GizmoActivationState::Active;
    }

    if (left_button && mouse_event.type() == MouseEvent::Type::ButtonUp && m_dragging) {
        m_dragging = false;
        return GizmoActivationState::Done;
    }

    return GizmoActivationState::Inactive;
}

void MeshFixGizmo::update_hover(const Scene::GizmoEventContext& ctx)
{
    const Scene::Ray ray = scene().camera().ray_at(ctx.screen_mouse_x(), ctx.screen_mouse_y());

    Hover hover;
    double hit_distance             = 0.;
    const Transform3d world_to_mesh = m_world_trafo.inverse();
    const Vec3d origin              = world_to_mesh * ray.origin;
    const Vec3d direction           = (world_to_mesh.linear() * ray.direction).normalized();
    const std::optional<Biz::Utils::MeshRaycaster::UnprojectResult> hit =
        Biz::Utils::MeshRaycaster::
            unproject_on_mesh(m_pick_mesh->aabb_mesh(), ray, m_world_trafo, std::nullopt, false);
    if (hit.has_value()) {
        hover.face   = int(hit->facet_idx);
        hover.point  = hit->position.cast<float>();
        hit_distance = (hit->position - origin).dot(direction);
    }
    if (m_tool == MeshFixTool::FillHole)
        hover.hole = m_session->hole_at_ray(origin, direction, hover.face, hit_distance);

    const bool face_changed = hover.face != m_hover.face;
    const bool hole_changed = hover.hole != m_hover.hole;
    const bool moved        = hover.face >= 0 && hover.point != m_hover.point;
    m_hover                 = hover;

    if (hole_changed)
        refresh_hole();
    if (face_changed)
        refresh_cursor_thickness();
    if (m_tool == MeshFixTool::MarkBrush) {
        if (face_changed || moved)
            refresh_cursor();
    } else if (face_changed) {
        update_preview();
    }
}

void MeshFixGizmo::update_preview()
{
    if (!m_session)
        return;
    const std::optional<MeshFix::MarkTool> mark_tool = to_mark_tool(m_tool);
    std::vector<bool> preview;
    if (mark_tool.has_value() && m_tool != MeshFixTool::MarkBrush && m_hover.face >= 0) {
        // The shell or surface does not change while the cursor stays inside. The plane does, it is
        // measured from the clicked triangle.
        const bool keep = (m_tool == MeshFixTool::MarkShell || m_tool == MeshFixTool::MarkSurface)
            && size_t(m_hover.face) < m_preview.size()
            && m_preview[m_hover.face];
        if (keep)
            return;
        preview = m_session->region(*mark_tool, m_hover.face, m_hover.point, mark_params());
    }
    if (preview.empty() && m_preview.empty())
        return;
    m_preview = std::move(preview);
    refresh_marking();
}

MeshFix::MarkParams MeshFixGizmo::mark_params() const
{
    MeshFix::MarkParams params;
    params.surface_angle_deg = float(m_surface_angle_deg);
    params.plane_angle_deg   = float(m_plane_angle_deg);
    params.plane_distance    = float(PlaneDistance / m_world_scale);
    // The brush is a sphere in the world like its cursor, also on a non-uniformly scaled part.
    params.brush_radius = float(m_brush_radius);
    params.brush_trafo  = m_world_trafo;
    return params;
}

void MeshFixGizmo::apply_marking_at_hover(bool add)
{
    const std::optional<MeshFix::MarkTool> mark_tool = to_mark_tool(m_tool);
    if (!mark_tool.has_value() || m_hover.face < 0)
        return;
    const MeshFix::FaceMask region =
        m_session->region(*mark_tool, m_hover.face, m_hover.point, mark_params());
    if (m_session->mark(region, add)) {
        refresh_marking();
        refresh_dialog();
    }
}

void MeshFixGizmo::brush_stroke_to(const Vec2d& screen_position)
{
    // Mark along the path from the previous mouse position, a fast stroke would leave gaps.
    const Vec2d from                 = m_last_brush_position;
    m_last_brush_position            = screen_position;
    const MeshFix::MarkParams params = mark_params();
    const size_t steps = std::max<size_t>(1, size_t(std::ceil((screen_position - from).norm())));
    // Triangles under the brush along the path, marked together.
    MeshFix::FaceMask stroke;
    for (size_t step = 1; step <= steps; ++step) {
        const Vec2d position = from + (screen_position - from) * (double(step) / double(steps));
        const Scene::Ray ray = scene().camera().ray_at(position.x(), position.y());
        const std::optional<Biz::Utils::MeshRaycaster::UnprojectResult> hit =
            Biz::Utils::MeshRaycaster::unproject_on_mesh(
                m_pick_mesh->aabb_mesh(),
                ray,
                m_world_trafo,
                std::nullopt,
                false
            );
        if (!hit.has_value())
            continue;
        // Spheres half the radius apart cover the path, marking at every pixel would be slow.
        const Vec3d world_point = m_world_trafo * hit->position;
        if (step < steps
            && m_last_brush_point.has_value()
            && (world_point - *m_last_brush_point).norm() < 0.5 * m_brush_radius)
            continue;
        m_last_brush_point = world_point;
        if (stroke.empty())
            stroke.assign(m_session->mesh().indices.size(), false);
        m_session
            ->add_brush_region(stroke, int(hit->facet_idx), hit->position.cast<float>(), params);
    }
    if (!stroke.empty() && m_session->mark(stroke, m_drag_adds)) {
        refresh_marking();
        refresh_dialog();
    }
}

void MeshFixGizmo::set_tool(MeshFixTool tool)
{
    m_tool       = tool;
    m_hover.hole = -1;
    m_preview.clear();
    if (m_tool == MeshFixTool::MarkSurface || m_tool == MeshFixTool::MarkPlane)
        m_dialog->set_angle(
            m_tool == MeshFixTool::MarkPlane ? m_plane_angle_deg : m_surface_angle_deg
        );
    if (m_session) {
        refresh_marking();
        refresh_hole();
        refresh_cursor();
    }
}

void MeshFixGizmo::push_dialog_state()
{
    m_dialog->set_tool(m_tool);
    m_dialog->set_angle(m_tool == MeshFixTool::MarkPlane ? m_plane_angle_deg : m_surface_angle_deg);
    m_dialog->set_brush_radius(m_brush_radius);
    m_dialog->set_fill_mode(m_fill_mode);
    m_dialog->set_stitch_tolerance(m_stitch_tolerance);
    m_dialog->set_show_problems(m_show_problems);
    m_dialog->set_min_wall(m_min_wall);
    m_dialog->set_show_thickness(m_show_thickness);
}

void MeshFixGizmo::refresh_mesh()
{
    // The triangles were renumbered, the picking and the hover are stale. The previous picking
    // mesh is released after the mesh node stops referencing it.
    const std::unique_ptr<Scene::TriangleMesh> previous_pick_mesh = std::exchange(
        m_pick_mesh,
        std::make_unique<Scene::TriangleMesh>(indexed_triangle_set(m_session->mesh()))
    );
    m_renderer->set_mesh_aabb(&m_pick_mesh->aabb_mesh());
    m_hover = {};
    m_preview.clear();
    refresh_marking();
    refresh_mesh_edges();
    refresh_hole();
    refresh_cursor();
    refresh_dialog();
    refresh_cursor_thickness();
}

void MeshFixGizmo::refresh_marking()
{
    if (!m_session || !m_renderer->exists())
        return;
    using FaceCategory                  = MeshFixRenderer::FaceCategory;
    const indexed_triangle_set& its     = m_session->mesh();
    const std::vector<bool>& marked     = m_session->marked();
    const std::vector<bool>& flipped    = m_session->flipped();
    const std::vector<bool>& noise      = m_session->noise();
    const std::vector<bool>& crossing   = m_session->intersecting();
    const std::vector<float>& thickness = m_session->thickness();
    const bool show_thickness           = m_show_thickness && m_session->thickness_analyzed();
    // The same limit as for marking the thin walls.
    const float min_wall = float(m_min_wall);
    std::vector<FaceCategory> categories(its.indices.size(), FaceCategory::Normal);
    for (size_t face = 0; face < categories.size(); ++face) {
        if (marked[face])
            categories[face] = FaceCategory::Marked;
        else if (face < m_preview.size() && m_preview[face])
            categories[face] = FaceCategory::Preview;
        else if (show_thickness && thickness[face] < min_wall) {
            // Shades from the thinnest walls to the walls close to the minimum.
            const size_t shade = std::min(
                size_t(thickness[face] / min_wall * float(MeshFixColors::ThinWallShades)),
                MeshFixColors::ThinWallShades - 1
            );
            categories[face] = FaceCategory(size_t(FaceCategory::ThinWall) + shade);
        } else if (m_show_problems && flipped[face])
            categories[face] = FaceCategory::Flipped;
        else if (m_show_problems && crossing[face])
            categories[face] = FaceCategory::Intersecting;
        else if (m_show_problems && noise[face])
            categories[face] = FaceCategory::Noise;
    }
    m_renderer->update_mesh(its, categories, flipped);
}

void MeshFixGizmo::refresh_mesh_edges()
{
    if (!m_session)
        return;
    std::vector<Vec3f> open_edges;
    std::vector<Vec3f> non_manifold_edges;
    if (m_show_problems) {
        const indexed_triangle_set& its       = m_session->mesh();
        const MeshFix::MeshTopology& topology = m_session->topology();
        for (size_t edge_idx = 0; edge_idx < topology.edges_count(); ++edge_idx) {
            const MeshFix::EdgeKind kind = topology.edge_kind(int(edge_idx));
            std::vector<Vec3f>* lines    = kind == MeshFix::EdgeKind::Open ?
                   &open_edges :
                   kind == MeshFix::EdgeKind::NonManifold ?
                   &non_manifold_edges :
                   nullptr;
            if (lines == nullptr)
                continue;
            const Domain::Index2& vertices = topology.edge_vertices(int(edge_idx));
            lines->push_back(its.vertices[vertices[0]]);
            lines->push_back(its.vertices[vertices[1]]);
        }
    }
    m_renderer->update_edges(open_edges, non_manifold_edges);
}

void MeshFixGizmo::refresh_hole()
{
    if (!m_session)
        return;
    std::vector<Vec3f> hole_edges;
    if (m_tool == MeshFixTool::FillHole
        && m_hover.hole >= 0
        && size_t(m_hover.hole) < m_session->holes().size())
    {
        const std::vector<int>& vertices = m_session->holes()[m_hover.hole].vertices;
        for (size_t i = 0; i < vertices.size(); ++i) {
            hole_edges.push_back(m_session->mesh().vertices[vertices[i]]);
            hole_edges.push_back(m_session->mesh().vertices[vertices[(i + 1) % vertices.size()]]);
        }
    }
    m_renderer->update_hole(hole_edges);
}

void MeshFixGizmo::refresh_cursor()
{
    std::optional<Vec3d> position;
    if (m_session && m_tool == MeshFixTool::MarkBrush && m_hover.face >= 0)
        position = m_world_trafo * m_hover.point.cast<double>();
    m_renderer->update_cursor(position, m_brush_radius);
}

void MeshFixGizmo::refresh_dialog()
{
    if (!m_session)
        return;
    const MeshFix::MeshDiagnostics& diagnostics = m_session->diagnostics();
    MeshFixStats stats;
    stats.triangles          = diagnostics.faces;
    stats.marked             = m_session->marked_count();
    stats.open_edges         = diagnostics.open_edges;
    stats.non_manifold_edges = diagnostics.non_manifold_edges;
    stats.holes              = diagnostics.holes;
    stats.flipped            = diagnostics.flipped_faces;
    stats.shells             = diagnostics.shells;
    stats.noise_shells       = diagnostics.noise_shells;
    if (m_session->intersections_detected())
        stats.intersecting = diagnostics.intersecting_faces;
    if (m_session->thickness_analyzed()) {
        const std::vector<float>& thickness = m_session->thickness();
        // The same limit as for marking the thin walls.
        const float min_wall = float(m_min_wall);
        stats.thin_faces     = size_t(
            std::count_if(
                thickness.begin(),
                thickness.end(),
                [min_wall](float value) { return value < min_wall; }
            )
        );
        if (!thickness.empty())
            stats.thinnest_wall = *std::min_element(thickness.begin(), thickness.end());
    }
    m_dialog->set_stats(stats);
    // The notice explains the last refused edit, the user moved on.
    m_dialog->set_notice({});
}

void MeshFixGizmo::refresh_cursor_thickness()
{
    std::optional<float> thickness;
    if (m_session
        && m_session->thickness_analyzed()
        && m_hover.face >= 0
        && size_t(m_hover.face) < m_session->thickness().size())
        thickness = m_session->thickness()[m_hover.face];
    m_dialog->set_cursor_thickness(thickness);
}

void MeshFixGizmo::commit(Biz::UndoSnapshotType snapshot_type, indexed_triangle_set split_part)
{
    using Biz::Scene::SceneInteractor;
    SceneInteractor& scene_interactor = m_project_interactor.scene_interactor();
    const Domain::Project& project    = m_project_interactor.selected_project();

    // An object made of a single part stays on the bed, even if the edit changes its lowest point.
    const Domain::ModelObject* object = project.find_object_by_id(m_target->object_id);
    const bool single_part =
        object != nullptr && object->volumes.size() == 1 && split_part.indices.empty();
    const std::optional<Biz::Scene::SelectionExtents> extents =
        scene_interactor.selection_bounding_box();
    const bool keep_on_bed = single_part && extents.has_value() && !extents->is_floating();

    m_committing = true;
    SceneInteractor::VolumeMeshReplacements replacements;
    replacements.push_back(
        SceneInteractor::VolumeMeshReplacement{
            *m_target,
            Biz::Algorithms::TriangleMesh::construct(indexed_triangle_set(m_session->mesh()))
        }
    );
    // The volume gets a new id.
    const Domain::ElementRefs updated =
        scene_interactor.change_volume_meshes(std::move(replacements));
    if (!updated.empty())
        m_target = updated.front();

    if (!updated.empty() && !split_part.indices.empty()) {
        const Domain::ElementRef target = *m_target;
        scene_interactor.add_volume(
            m_project_id,
            target.instance_id,
            [&target, &split_part](Domain::ModelObject& model_object) -> Domain::ModelVolume*
            {
                const auto source = std::find_if(
                    model_object.volumes.begin(),
                    model_object.volumes.end(),
                    [&target](const Domain::ModelVolume* volume)
                    { return volume->id().id == target.volume_id; }
                );
                ASSERT(source != model_object.volumes.end());
                const Domain::ModelVolume& source_volume = **source;
                Domain::ModelVolume* volume              = Biz::Algorithms::ModelObject::add_volume(
                    &model_object,
                    source_volume,
                    Biz::Algorithms::TriangleMesh::construct(std::move(split_part))
                );
                // TRN Mesh fix: Name of a part made of the triangles split off another part.
                volume->name   = source_volume.name + " " + Biz::_u8L("(split)");
                volume->source = Domain::ModelVolume::Source();
                return volume;
            }
        );
        // Adding the part selected it, keep fixing the original part.
        scene_interactor.set_object_selection(
            Biz::Scene::ObjectSelection{Biz::Scene::SelectionMode::Volume, {target}}
        );
    }

    if (keep_on_bed) {
        const std::optional<Biz::Scene::SelectionExtents> new_extents =
            scene_interactor.selection_bounding_box();
        if (new_extents.has_value() && new_extents->is_floating()) {
            Domain::SquareMatrix4d drop = Domain::SquareMatrix4d::Identity();
            drop.col(3).z()             = -new_extents->min_z();
            scene_interactor.transform_selection(drop);
        }
    }
    m_committing = false;
    m_project_interactor.undo_provider().take_snapshot(snapshot_type);

    // The gizmo manager deactivates the tool when the edit leaves it no part to fix.
    if (!m_active) {
        m_target.reset();
        return;
    }
    if (updated.empty()) {
        if (m_gizmo_controller != nullptr)
            m_gizmo_controller->deactivate_current_tool();
        return;
    }
    // The edit removed the painting of the part, if there was any.
    m_dialog->set_painting_warning(false);
    // The presenter re-created the volume nodes under the new volume id.
    update_world_transform();
    hide_target_nodes();
    refresh_mesh();
}

void MeshFixGizmo::delete_marked()
{
    if (!m_session || m_session->marked_count() == 0)
        return;
    if (m_session->delete_marked() > 0)
        commit(Biz::UndoSnapshotType::MeshFixDeleteTriangles);
    else
        m_dialog->set_notice(
            Biz::_u8L(
                "At least two triangles of the part have to remain. To remove the whole part, delete it in the object list."
            )
        );
}

void MeshFixGizmo::flip_marked()
{
    if (m_session && m_session->flip_marked() > 0)
        commit(Biz::UndoSnapshotType::MeshFixFlipTriangles);
}

void MeshFixGizmo::split_marked()
{
    if (!m_session || m_session->marked_count() == 0)
        return;
    indexed_triangle_set part = m_session->split_marked();
    if (!part.indices.empty())
        commit(Biz::UndoSnapshotType::MeshFixSplitPart, std::move(part));
    else
        m_dialog->set_notice(
            Biz::_u8L(
                "Only a piece with volume can be split off, and at least two triangles of the part have to remain."
            )
        );
}

void MeshFixGizmo::fill_hovered_hole()
{
    if (m_session && m_session->fill_hole(m_hover.hole, to_fill_mode(m_fill_mode)) > 0)
        commit(Biz::UndoSnapshotType::MeshFixFillHoles);
}

void MeshFixGizmo::fill_all_holes()
{
    if (m_session && m_session->fill_all_holes(to_fill_mode(m_fill_mode)) > 0)
        commit(Biz::UndoSnapshotType::MeshFixFillHoles);
}

void MeshFixGizmo::unify_normals()
{
    if (m_session && m_session->unify_normals() > 0)
        commit(Biz::UndoSnapshotType::MeshFixUnifyNormals);
}

void MeshFixGizmo::stitch()
{
    if (m_session && m_session->stitch(float(m_stitch_tolerance / m_world_scale)) > 0)
        commit(Biz::UndoSnapshotType::MeshFixStitch);
}

void MeshFixGizmo::remove_noise_shells()
{
    if (m_session && m_session->remove_noise_shells() > 0)
        commit(Biz::UndoSnapshotType::MeshFixRemoveNoiseShells);
}

void MeshFixGizmo::auto_fix()
{
    if (!m_session)
        return;
    MeshFix::AutoFixParams params;
    params.stitch_tolerance = float(m_stitch_tolerance / m_world_scale);
    params.hole_fill.mode   = to_fill_mode(m_fill_mode);
    if (m_session->auto_fix(params).changed())
        commit(Biz::UndoSnapshotType::MeshFixAutoFix);
}

void MeshFixGizmo::check_intersections()
{
    if (!m_session)
        return;
    m_session->detect_intersections();
    refresh_marking();
    refresh_dialog();
}

void MeshFixGizmo::analyze_thickness()
{
    if (!m_session)
        return;
    // Measured in the world, where the minimum wall applies.
    m_session->analyze_thickness(m_world_trafo);
    m_show_thickness = true;
    m_dialog->set_show_thickness(true);
    refresh_marking();
    refresh_dialog();
    refresh_cursor_thickness();
}

void MeshFixGizmo::mark_thin_walls()
{
    if (!m_session)
        return;
    if (!m_session->thickness_analyzed())
        analyze_thickness();
    m_session->mark_thin_faces(float(m_min_wall));
    refresh_marking();
    refresh_dialog();
}

} // namespace Slic3r::App::Plater

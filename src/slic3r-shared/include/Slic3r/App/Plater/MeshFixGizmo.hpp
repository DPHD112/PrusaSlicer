#pragma once

#include <admesh/stl.h>

#include <memory>
#include <optional>
#include <vector>

#include "Slic3r/App/Plater/MeshFixDialog.hpp"
#include "Slic3r/App/Render/Device.hpp"
#include "Slic3r/App/Scene/GeometryDataFactory.hpp"
#include "Slic3r/App/Scene/IGizmo.hpp"
#include "Slic3r/App/Scene/ISceneChangedListener.hpp"
#include "Slic3r/App/Scene/IThumbnailRenderListener.hpp"
#include "Slic3r/App/Yoga/Item.hpp"
#include "Slic3r/Biz/IUndoProvider.hpp"
#include "Slic3r/Biz/ProjectInteractor.hpp"
#include "Slic3r/Biz/Scene/SceneInteractor.hpp"
#include "Slic3r/Domain/ElementRef.hpp"
#include "Slic3r/Domain/Types.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix {
class Session;
struct MarkParams;
} // namespace Slic3r::Biz::Algorithms::MeshFix

namespace Slic3r::App::Scene {
class TriangleMesh;
} // namespace Slic3r::App::Scene

namespace Slic3r::App::Plater {

class MeshFixRenderer;
class PlaterScenePresenter;

/**
 * @brief Tool for manual and automatic fixing of a part mesh, in the spirit of Materialise Magics.
 *
 * The user marks triangles by clicking (single triangles, shells, smooth surfaces, planes or with
 * a brush) and deletes, flips or splits off the marked triangles, fills holes by clicking them or
 * runs the automatic repairs. Mesh errors (bad edges, holes, flipped triangles, noise shells,
 * intersecting triangles) are highlighted and counted in the dialog.
 *
 * Every edit is applied to the model right away as an undoable step, the marking lives only in
 * the tool. The tool works on a single model part: the selected part, or the only part of the
 * selected object.
 */
class MeshFixGizmo :
    public Scene::IToolGizmo,
    public App::Scene::ISceneChangedListener,
    public Biz::Scene::ISceneChangedListener,
    public Biz::Scene::ISceneSelectionChangedListener,
    public Scene::IThumbnailRenderListener
{
public:
    MeshFixGizmo(
        Render::Device& device,
        Scene::GeometryDataFactory& data_factory,
        Biz::ProjectInteractor& project_interactor,
        PlaterScenePresenter& scene_presenter
    );
    ~MeshFixGizmo() override;

    /**
     * @name Implementation of IGizmo interface
     * @{
     */
    Scene::GizmoActivationState on_mouse(Scene::GizmoEventContext& ctx, bool only_active) override;
    void register_commands(Platform::CommandRegistry& registry) override;
    bool disable_object_selection() const override;
    /**@}*/

    /**
     * @name Implementation of IToolGizmo interface
     * @{
     */
    void on_activated() override;
    void on_deactivated() override;
    void on_project_activated(size_t new_project_id) override;
    void on_project_deactivated(size_t old_project_id) override;
    Scene::ToolType type() const override;
    bool enabled() const override;
    std::unique_ptr<GizmoWindow> release_ui_window() override;
    void provide_gizmo_controller(Scene::IGizmoController& gizmo_controller) override;
    /**@}*/

    /**
     * @name Implementation of App::Scene::ISceneChangedListener interface
     * Keeps the nodes of the edited volume hidden when the presenter re-creates them.
     * @{
     */
    void on_node_added(App::Scene::Node* node) override;
    void on_node_removed(App::Scene::Node* node) override;
    /**@}*/

    /**
     * @name Implementation of Biz::Scene::ISceneChangedListener interface
     * @{
     */
    void on_model_reloaded(Domain::SelectionId project_id) override;
    void on_volume_mesh_changed(
        Domain::SelectionId project_id,
        const Domain::ElementRefs& volumes
    ) override;
    void on_volume_transformed(
        Domain::SelectionId project_id,
        const Domain::ElementRefs& elements,
        Biz::Scene::TransformState state,
        const Biz::BedTrackingChanges& bed_tracking_changes
    ) override;
    void on_instance_transformed(
        Domain::SelectionId project_id,
        const Domain::ElementRefs& elements,
        Biz::Scene::TransformState state,
        const Biz::BedTrackingChanges& bed_tracking_changes
    ) override;
    /**@}*/

    /**
     * @name Implementation of ISceneSelectionChangedListener interface
     * @{
     */
    void on_scene_selection_changed(
        Domain::SelectionId project_id,
        const Biz::Scene::ObjectSelection& selection
    ) override;
    /**@}*/

    /**
     * @name Implementation of IThumbnailRenderListener interface
     * @{
     */
    void on_thumbnail_render_begin() override;
    void on_thumbnail_render_end() override;
    /**@}*/

private:
    struct Hover
    {
        // Triangle under the cursor, -1 for none.
        int face{-1};
        // Point under the cursor in mesh coordinates.
        Domain::Vec3f point{Domain::Vec3f::Zero()};
        // Hole the cursor points at, -1 for none.
        int hole{-1};
    };

    std::optional<Domain::ElementRef> find_target() const;
    Scene::Scene& scene();

    void activate();
    void release();
    // Load the mesh of the target volume into the tool.
    void load_target();
    void update_world_transform();
    void hide_target_nodes();
    void restore_hidden_nodes();
    void add_listeners();
    void remove_listeners();

    void update_hover(const Scene::GizmoEventContext& ctx);
    void update_preview();
    Biz::Algorithms::MeshFix::MarkParams mark_params() const;
    void apply_marking_at_hover(bool add);
    // Continue the brush stroke to the mouse position.
    void brush_stroke_to(const Domain::Vec2d& screen_position);
    void set_tool(MeshFixTool tool);
    void push_dialog_state();

    // Refresh the scene and the dialog after the mesh or its marking changed.
    void refresh_mesh();
    void refresh_marking();
    void refresh_mesh_edges();
    void refresh_hole();
    void refresh_cursor();
    void refresh_dialog();
    void refresh_cursor_thickness();

    /**
     * @brief Replace the volume mesh by the edited one and take an undo snapshot.
     * @param split_part Triangles split off the mesh, added as a new part of the object.
     */
    void commit(Biz::UndoSnapshotType snapshot_type, indexed_triangle_set split_part = {});

    void delete_marked();
    void flip_marked();
    void split_marked();
    void fill_hovered_hole();
    void fill_all_holes();
    void unify_normals();
    void stitch();
    void remove_noise_shells();
    void auto_fix();
    void check_intersections();
    void analyze_thickness();
    void mark_thin_walls();

    Biz::ProjectInteractor& m_project_interactor;
    PlaterScenePresenter& m_scene_presenter;
    Scene::IGizmoController* m_gizmo_controller{nullptr};

    std::unique_ptr<MeshFixRenderer> m_renderer;
    Yoga::Passthrough<MeshFixDialog> m_dialog;

    bool m_active{false};
    // Project the tool is active in, its scene holds the nodes of the tool.
    Domain::SelectionId m_project_id{0};
    std::optional<Domain::ElementRef> m_target;
    Domain::Transform3d m_world_trafo{Domain::Transform3d::Identity()};
    // Average scale of the world transformation, to convert lengths to mesh units.
    double m_world_scale{1.};
    std::unique_ptr<Biz::Algorithms::MeshFix::Session> m_session;
    // Copy of the edited mesh with an AABB tree for picking.
    std::unique_ptr<Scene::TriangleMesh> m_pick_mesh;
    std::vector<size_t> m_hidden_node_ids;
    // Set while the tool changes the model, to ignore the notifications it causes.
    bool m_committing{false};

    MeshFixTool m_tool{MeshFixTool::MarkTriangle};
    double m_surface_angle_deg{20.};
    double m_plane_angle_deg{1.};
    double m_brush_radius{2.};
    MeshFixFillMode m_fill_mode{MeshFixFillMode::Smooth};
    double m_stitch_tolerance{0.01};
    bool m_show_problems{true};
    // Walls thinner than this are highlighted, in mm.
    double m_min_wall{1.};
    bool m_show_thickness{true};

    Hover m_hover;
    // Triangles a click would mark, shown under the cursor.
    std::vector<bool> m_preview;
    bool m_dragging{false};
    bool m_drag_adds{true};
    // Mouse position of the last brush stroke event.
    Domain::Vec2d m_last_brush_position{Domain::Vec2d::Zero()};
    // Last world position the brush marked at.
    std::optional<Domain::Vec3d> m_last_brush_point;
};

} // namespace Slic3r::App::Plater

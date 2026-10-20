#pragma once

#include <admesh/stl.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "Slic3r/App/Plater/MeshFixColors.hpp"
#include "Slic3r/App/Plater/PaintOnGizmoHelper.hpp"
#include "Slic3r/App/Render/Device.hpp"
#include "Slic3r/App/Render/Geometry.hpp"
#include "Slic3r/App/Render/Material.hpp"
#include "Slic3r/App/Scene/GeometryDataFactory.hpp"
#include "Slic3r/App/Scene/IRenderLayerObject.hpp"
#include "Slic3r/App/Scene/Node.hpp"
#include "Slic3r/App/Scene/Scene.hpp"
#include "Slic3r/Domain/Color.hpp"
#include "Slic3r/Domain/Types.hpp"

namespace Slic3r {
class AABBMesh;
} // namespace Slic3r

namespace Slic3r::App::Plater {

/**
 * @brief Tag of the scene nodes created by the mesh fix tool.
 */
struct MeshFixNodeTag
{};

/**
 * @brief Scene nodes of the mesh fix tool.
 *
 * Draws the edited mesh colored by the state of its triangles, the open and non-manifold edges,
 * the boundary of the hole under the cursor and the brush cursor. The original volume is hidden
 * by the tool while it is active.
 */
class MeshFixRenderer
{
public:
    enum class FaceCategory : uint8_t
    {
        Normal,
        Marked,
        Preview,
        Flipped,
        Intersecting,
        Noise,
        // Walls thinner than the minimum, MeshFixColors::ThinWallShades shades from the thinnest.
        ThinWall,
        Count = ThinWall + MeshFixColors::ThinWallShades
    };

    MeshFixRenderer(Render::Device& device, Scene::GeometryDataFactory& data_factory);

    /**
     * @brief Create the scene nodes under the scene root.
     * @param world_trafo Transformation of the mesh coordinates to the world.
     */
    void create(Scene::Scene& scene, const Domain::Transform3d& world_trafo);
    /**
     * @brief Remove the scene nodes from the scene.
     */
    void destroy(Scene::Scene& scene);

    bool exists() const
    {
        return m_main_node != nullptr;
    }

    void set_transform(const Domain::Transform3d& world_trafo);
    void set_enabled(bool enabled);

    /**
     * @brief Rebuild the triangles, one category per triangle of the mesh.
     * @param flipped Triangles facing inwards, drawn reversed to be visible from outside.
     */
    void update_mesh(
        const indexed_triangle_set& its,
        const std::vector<FaceCategory>& categories,
        const std::vector<bool>& flipped
    );
    /**
     * @brief Bounding volume of the mesh, used for the camera clipping planes. It has to stay
     * alive until it is replaced or the nodes are destroyed.
     */
    void set_mesh_aabb(const AABBMesh* aabb);
    /**
     * @brief Rebuild the edge lines, pairs of end points in mesh coordinates.
     */
    void update_edges(
        const std::vector<Domain::Vec3f>& open_edges,
        const std::vector<Domain::Vec3f>& non_manifold_edges
    );
    /**
     * @brief Show the boundary of the hole under the cursor, hide it when empty.
     */
    void update_hole(const std::vector<Domain::Vec3f>& hole_edges);
    /**
     * @brief Show the brush cursor at a world position, hide it when there is none.
     */
    void update_cursor(const std::optional<Domain::Vec3d>& world_position, double radius);

private:
    std::unique_ptr<Render::Geometry> create_lines_geometry(
        const std::vector<std::pair<const std::vector<Domain::Vec3f>*, Domain::ColorRGBA>>& lines
    ) const;
    Render::Material lines_material() const;
    Scene::Node* add_child_node(
        const char* name,
        const Render::Geometry* geometry,
        const Render::Material& material,
        Scene::RenderLayerId layer
    );
    // Show the geometry in the node, created on demand. A null geometry hides the node.
    void set_node_geometry(
        Scene::Node*& node,
        std::unique_ptr<Render::Geometry>& owner,
        std::unique_ptr<Render::Geometry> geometry,
        const char* name,
        const Render::Material& material,
        Scene::RenderLayerId layer
    );

    Render::Device& m_device;
    Scene::GeometryDataFactory& m_data_factory;
    Scene::Scene* m_scene{nullptr};

    Domain::Transform3d m_world_trafo{Domain::Transform3d::Identity()};
    const AABBMesh* m_mesh_aabb{nullptr};

    Scene::Node* m_main_node{nullptr};
    Scene::Node* m_mesh_node{nullptr};
    Scene::Node* m_edges_node{nullptr};
    Scene::Node* m_hole_node{nullptr};
    Scene::Node* m_cursors_node{nullptr};

    std::unique_ptr<Render::Geometry> m_mesh_geometry;
    std::unique_ptr<Render::Geometry> m_edges_geometry;
    std::unique_ptr<Render::Geometry> m_hole_geometry;

    SphereCursorRenderWrapper m_brush_cursor;
};

} // namespace Slic3r::App::Plater

#include "Slic3r/App/Plater/MeshFixRenderer.hpp"

#include <array>
#include <cfloat>
#include <utility>

#include "Slic3r/App/Plater/MeshFixColors.hpp"
#include "Slic3r/App/Plater/PlaterSceneLayer.hpp"
#include "Slic3r/App/Render/GeometryBuilder.hpp"
#include "Slic3r/App/Scene/AabbRaycastNodeComponent.hpp"
#include "Slic3r/App/Scene/MeshRenderNodeComponent.hpp"
#include "Slic3r/App/Scene/NodeBuilder.hpp"
#include "Slic3r/Assert.hpp"
#include "Slic3r/Domain/Model.hpp"

using Slic3r::Domain::ColorRGBA;
using Slic3r::Domain::SquareMatrix3f;
using Slic3r::Domain::Transform3d;
using Slic3r::Domain::Vec2f;
using Slic3r::Domain::Vec3d;
using Slic3r::Domain::Vec3f;
using Slic3r::Domain::Vec4f;

namespace Slic3r::App::Plater {

namespace {

ColorRGBA category_color(MeshFixRenderer::FaceCategory category)
{
    using FaceCategory = MeshFixRenderer::FaceCategory;
    switch (category) {
    case FaceCategory::Marked:
        return MeshFixColors::marked();
    case FaceCategory::Preview:
        return MeshFixColors::preview();
    case FaceCategory::Flipped:
        return MeshFixColors::flipped();
    case FaceCategory::Intersecting:
        return MeshFixColors::intersecting();
    case FaceCategory::Noise:
        return MeshFixColors::noise();
    case FaceCategory::Normal:
    case FaceCategory::Count:
        break;
    }
    return MeshFixColors::mesh();
}

Render::Material mesh_material(Render::Device& device, const Transform3d& world_trafo)
{
    // Cut at the bed like the paint tools, the picking ignores the parts below the bed as well.
    const Vec4f clipping_plane{0.f, 0.f, 1.f, FLT_MAX};
    const Vec2f z_range{Domain::SINKING_Z_THRESHOLD, FLT_MAX};
    const SquareMatrix3f world_normal_matrix =
        world_trafo.linear().inverse().transpose().cast<float>();
    return Render::Material{}
        .set_shader(device.context().shader_manager().shader("mm_gouraud"))
        .set_uniform("clipping_plane", clipping_plane)
        .set_uniform("z_range", z_range)
        .set_uniform("overhang.enabled", false)
        .set_uniform("overhang.world_normal_matrix", world_normal_matrix)
        .set_uniform("overhang.max_normal_z", -1.f);
}

Scene::MeshRenderNodeComponent* mesh_component(Scene::Node* node)
{
    return dynamic_cast<Scene::MeshRenderNodeComponent*>(node->render_component());
}

} // namespace

MeshFixRenderer::MeshFixRenderer(Render::Device& device, Scene::GeometryDataFactory& data_factory) :
    m_device(device),
    m_data_factory(data_factory)
{}

void MeshFixRenderer::create(Scene::Scene& scene, const Transform3d& world_trafo)
{
    ASSERT(m_main_node == nullptr);
    m_scene       = &scene;
    m_world_trafo = world_trafo;

    Scene::NodeBuilder main_node_builder{scene};
    main_node_builder.set_debug_name("MeshFixGizmo - Main node").set_tag(MeshFixNodeTag{});
    std::unique_ptr<Scene::Node> main_node = main_node_builder.build();
    m_main_node                            = main_node.get();
    // A child of the scene root and not the selection root to avoid screen space rescaling.
    scene.add_child(main_node.release(), &scene.root());

    Scene::NodeBuilder cursors_node_builder{scene};
    cursors_node_builder.set_debug_name("MeshFixGizmo - Cursors node").set_tag(MeshFixNodeTag{});
    std::unique_ptr<Scene::Node> cursors_node = cursors_node_builder.build();
    m_cursors_node                            = cursors_node.get();
    scene.add_child(cursors_node.release(), m_main_node);

    m_brush_cursor.init_cursor_node(m_device, scene, *m_cursors_node, m_data_factory);
}

void MeshFixRenderer::destroy(Scene::Scene& scene)
{
    if (m_main_node != nullptr)
        scene.remove_child(m_main_node);
    m_main_node    = nullptr;
    m_mesh_node    = nullptr;
    m_edges_node   = nullptr;
    m_hole_node    = nullptr;
    m_cursors_node = nullptr;
    m_scene        = nullptr;
    m_mesh_aabb    = nullptr;
    m_mesh_geometry.reset();
    m_edges_geometry.reset();
    m_hole_geometry.reset();
}

void MeshFixRenderer::set_transform(const Transform3d& world_trafo)
{
    m_world_trafo = world_trafo;
    for (Scene::Node* node : {m_mesh_node, m_edges_node, m_hole_node})
        if (node != nullptr)
            node->set_local_transform(world_trafo);
    if (m_mesh_node != nullptr)
        mesh_component(m_mesh_node)->replace_material(mesh_material(m_device, world_trafo));
}

void MeshFixRenderer::set_enabled(bool enabled)
{
    if (m_main_node != nullptr)
        m_main_node->set_enabled(enabled);
}

Scene::Node* MeshFixRenderer::add_child_node(
    const char* name,
    const Render::Geometry* geometry,
    const Render::Material& material,
    Scene::RenderLayerId layer
)
{
    ASSERT(m_scene != nullptr && m_main_node != nullptr);
    Scene::NodeBuilder builder{*m_scene};
    builder.set_debug_name(name)
        .set_tag(MeshFixNodeTag{})
        .set_transform(m_world_trafo)
        .set_mesh(geometry, material, layer);
    std::unique_ptr<Scene::Node> node = builder.build();
    Scene::Node* node_ptr             = node.get();
    m_scene->add_child(node.release(), m_main_node);
    return node_ptr;
}

void MeshFixRenderer::set_node_geometry(
    Scene::Node*& node,
    std::unique_ptr<Render::Geometry>& owner,
    std::unique_ptr<Render::Geometry> geometry,
    const char* name,
    const Render::Material& material,
    Scene::RenderLayerId layer
)
{
    if (m_main_node == nullptr)
        return;
    if (!geometry) {
        // Nodes without draw commands must not be rendered.
        if (node != nullptr)
            node->set_enabled(false);
        return;
    }
    if (node == nullptr) {
        node  = add_child_node(name, geometry.get(), material, layer);
        owner = std::move(geometry);
        return;
    }
    // Swap the geometry first, the old one is released after the node does not reference it.
    mesh_component(node)->set_geometry(geometry.get());
    owner = std::move(geometry);
    node->set_enabled(true);
}

void MeshFixRenderer::update_mesh(
    const indexed_triangle_set& its,
    const std::vector<FaceCategory>& categories,
    const std::vector<bool>& flipped
)
{
    ASSERT(categories.size() == its.indices.size() && flipped.size() == its.indices.size());
    std::unique_ptr<Render::Geometry> geometry;
    if (!its.indices.empty()) {
        Render::GeometryBuilder<Render::VertexP3> builder;
        builder.reserve(its.vertices.size(), 3 * its.indices.size());
        for (const stl_vertex& vertex : its.vertices)
            builder.add_vertex(Render::VertexP3(vertex));

        // The index buffer covers all the triangles of a mesh without unreferenced vertices, which
        // is required by GeometryBuilder choosing the index type by the index count.
        std::array<std::vector<uint32_t>, size_t(FaceCategory::Count)> indices_per_category;
        for (size_t face = 0; face < its.indices.size(); ++face) {
            const stl_triangle_vertex_indices& indices = its.indices[face];
            std::vector<uint32_t>& bucket = indices_per_category[size_t(categories[face])];
            // Flipped triangles face inwards, they would be culled from outside without reversing
            // them, whatever color they are shown in.
            const bool reverse = flipped[face];
            bucket.push_back(uint32_t(indices[0]));
            bucket.push_back(uint32_t(reverse ? indices[2] : indices[1]));
            bucket.push_back(uint32_t(reverse ? indices[1] : indices[2]));
        }

        size_t offset = 0;
        for (size_t category = 0; category < indices_per_category.size(); ++category) {
            std::vector<uint32_t>& indices = indices_per_category[category];
            if (indices.empty())
                continue;
            const size_t count = indices.size();
            builder.add_indices(std::move(indices));
            builder.add_draw_command(
                Render::DrawCommand(
                    Render::PrimitiveType::Triangles,
                    offset,
                    count,
                    Render::Material()
                        .set_uniform("uniform_color", category_color(FaceCategory(category)))
                )
            );
            offset += count;
        }
        geometry = builder.build(m_device);
    }
    const bool created = m_mesh_node == nullptr;
    set_node_geometry(
        m_mesh_node,
        m_mesh_geometry,
        std::move(geometry),
        "MeshFixGizmo - Mesh node",
        mesh_material(m_device, m_world_trafo),
        Scene::RenderLayerId(PlaterSceneLayer::DocumentObjects)
    );
    if (created)
        set_mesh_aabb(m_mesh_aabb);
}

void MeshFixRenderer::set_mesh_aabb(const AABBMesh* aabb)
{
    m_mesh_aabb = aabb;
    // The hidden volume nodes no longer count, the camera has to see the edited mesh instead.
    if (m_mesh_node != nullptr)
        m_mesh_node->set_raycast_component(
            aabb != nullptr ? new Scene::AabbRaycastNodeComponent(aabb) : nullptr
        );
}

std::unique_ptr<Render::Geometry> MeshFixRenderer::create_lines_geometry(
    const std::vector<std::pair<const std::vector<Vec3f>*, ColorRGBA>>& lines
) const
{
    Render::GeometryBuilder<Render::VertexP3> builder;
    size_t offset = 0;
    for (const auto& [points, color] : lines) {
        if (points->empty())
            continue;
        for (const Vec3f& point : *points)
            builder.add_vertex(Render::VertexP3(point));
        builder.add_draw_command(
            Render::DrawCommand(
                Render::PrimitiveType::Lines,
                offset,
                points->size(),
                Render::Material()
                    .set_uniform("uniform_color", color)
                    .set_uniform("offset", 0.00001f)
            )
        );
        offset += points->size();
    }
    if (offset == 0)
        return nullptr;
    return builder.build(m_device);
}

Render::Material MeshFixRenderer::lines_material() const
{
    return Render::Material{}.set_shader(m_device.context().shader_manager().shader("mm_contour"));
}

void MeshFixRenderer::
    update_edges(const std::vector<Vec3f>& open_edges, const std::vector<Vec3f>& non_manifold_edges)
{
    set_node_geometry(
        m_edges_node,
        m_edges_geometry,
        create_lines_geometry(
            {{&open_edges, MeshFixColors::open_edge()},
             {&non_manifold_edges, MeshFixColors::non_manifold_edge()}}
        ),
        "MeshFixGizmo - Edges node",
        lines_material(),
        Scene::RenderLayerId(PlaterSceneLayer::DocumentObjects)
    );
}

void MeshFixRenderer::update_hole(const std::vector<Vec3f>& hole_edges)
{
    // Drawn over the model, so that the whole boundary of the hole is visible.
    set_node_geometry(
        m_hole_node,
        m_hole_geometry,
        create_lines_geometry({{&hole_edges, MeshFixColors::hole()}}),
        "MeshFixGizmo - Hole node",
        lines_material(),
        Scene::RenderLayerId(PlaterSceneLayer::ObjectAccessoriesOnTop)
    );
}

void MeshFixRenderer::update_cursor(const std::optional<Vec3d>& world_position, double radius)
{
    if (m_cursors_node == nullptr)
        return;
    if (world_position.has_value())
        m_brush_cursor.update_cursor_geometry(*world_position, radius, MeshFixColors::brush());
    m_brush_cursor.set_enabled(world_position.has_value());
}

} // namespace Slic3r::App::Plater

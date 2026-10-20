#pragma once

#include <admesh/stl.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

#include "Slic3r/Biz/Algorithms/MeshFix/Diagnostics.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/HoleFill.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/Marking.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/MeshTopology.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/Repair.hpp"
#include "Slic3r/Domain/Types.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix {

/**
 * @brief How a click on a triangle marks the mesh.
 */
enum class MarkTool
{
    // The clicked triangle.
    Triangle,
    // All triangles connected to the clicked one.
    Shell,
    // Smooth surface around the clicked triangle, bounded by edges sharper than the angle tolerance.
    Surface,
    // Planar region around the clicked triangle.
    Plane,
    // Triangles touching a sphere around the clicked point.
    Brush
};

struct MarkParams
{
    // Maximal angle between neighbor triangles of a surface.
    float surface_angle_deg{20.f};
    // Maximal deviation of a triangle normal from the normal of a plane.
    float plane_angle_deg{1.f};
    // Maximal distance of triangle vertices from a plane, in mesh units.
    float plane_distance{0.01f};
    // Brush radius, in the units of brush_trafo.
    float brush_radius{1.f};
    // Transformation of the mesh in which the brush is a sphere, the world transformation of a
    // part, which may be scaled non-uniformly.
    Domain::Transform3d brush_trafo{Domain::Transform3d::Identity()};
};

/**
 * @brief Interactive editing state of a single mesh: the mesh being fixed, its marked triangles
 * and the diagnostics of its errors, which are kept up to date after every edit.
 *
 * Triangle indices refer to mesh().indices. Edits which renumber the triangles clear the marking.
 */
class Session
{
public:
    explicit Session(indexed_triangle_set mesh);

    /**
     * @brief Replace the mesh being edited, for example after an undo. Clears the marking.
     */
    void reset(indexed_triangle_set mesh);

    const indexed_triangle_set& mesh() const
    {
        return m_mesh;
    }

    const MeshTopology& topology() const
    {
        return *m_topology;
    }

    const MeshDiagnostics& diagnostics() const
    {
        return m_diagnostics;
    }

    const std::vector<Hole>& holes() const
    {
        return m_holes;
    }

    const FaceMask& marked() const
    {
        return m_marked;
    }

    size_t marked_count() const
    {
        return m_marked_count;
    }

    // Triangles to flip to get a consistent outward orientation.
    const FaceMask& flipped() const
    {
        return m_flipped;
    }

    // Triangles of noise shells.
    const FaceMask& noise() const
    {
        return m_noise;
    }

    // Triangles intersecting other triangles, empty until detect_intersections() is called.
    const FaceMask& intersecting() const
    {
        return m_intersecting;
    }

    bool intersections_detected() const
    {
        return m_intersections_detected;
    }

    // Wall thickness of each triangle, see measure_thickness(). Empty until analyze_thickness()
    // is called.
    const std::vector<float>& thickness() const
    {
        return m_thickness;
    }

    bool thickness_analyzed() const
    {
        return m_thickness_analyzed;
    }

    /**
     * @brief Triangles a click with the tool would mark.
     * @param face Clicked triangle.
     * @param point Clicked point in mesh coordinates, the center of the brush.
     */
    FaceMask
    region(MarkTool tool, int face, const Domain::Vec3f& point, const MarkParams& params) const;
    /**
     * @brief Add the triangles a brush click would mark to the region. Lets a brush stroke collect
     * many spheres without a mask of the whole mesh for each of them.
     * @param region Mask with a flag for each triangle of the mesh.
     */
    void add_brush_region(
        FaceMask& region,
        int face,
        const Domain::Vec3f& point,
        const MarkParams& params
    ) const;

    /**
     * @brief Add the region to the marking, or remove it from the marking.
     * @return True if the marking changed.
     */
    bool mark(const FaceMask& region, bool add);
    void mark_all();
    void clear_marks();
    void invert_marks();
    void grow_marks();
    void shrink_marks();
    void mark_bad_edge_faces();
    void mark_flipped_faces();
    void mark_noise_shells();
    void mark_intersecting_faces();
    /**
     * @brief Mark the triangles thinner than the limit, nothing if the thickness was not analyzed.
     */
    void mark_thin_faces(float limit);

    /**
     * @brief Hole a ray points at, to fill it by a click.
     *
     * Prefers holes the ray passes through in front of the hit surface,
     * then the hole bordering the hit triangle.
     * @param origin Ray origin in mesh coordinates.
     * @param direction Ray direction in mesh coordinates.
     * @param hit_face Triangle hit by the ray, -1 for none.
     * @param hit_distance Distance of the hit along the normalized ray direction.
     * @return Index into holes(), -1 if there is no such hole.
     */
    int hole_at_ray(
        const Domain::Vec3d& origin,
        const Domain::Vec3d& direction,
        int hit_face,
        double hit_distance
    ) const;

    // Edits. All of them return the number of changed elements, zero if the mesh did not change.
    // The edits which remove triangles are refused if less than two triangles would remain.

    /**
     * @brief Delete the marked triangles.
     */
    size_t delete_marked();
    size_t flip_marked();
    /**
     * @brief Remove the marked triangles from the mesh. Refuses a flat piece, which would make a
     * part without volume.
     * @return The removed triangles, empty if none were removed.
     */
    indexed_triangle_set split_marked();
    size_t fill_hole(int hole_index, HoleFillMode mode);
    size_t fill_all_holes(HoleFillMode mode);
    size_t unify_normals();
    /**
     * @brief Stitch the open edges. A large tolerance may collapse a small open mesh, which is
     * refused.
     */
    size_t stitch(float tolerance);
    size_t remove_noise_shells(const NoiseShellParams& params = {});
    /**
     * @brief Run all repairs. They may remove all triangles, for example of a surface made of pairs
     * of opposite triangles, which is refused.
     */
    AutoFixReport auto_fix(const AutoFixParams& params = {});

    /**
     * @brief Find intersecting triangles, an expensive check which is not run after every edit.
     */
    void detect_intersections();
    /**
     * @brief Measure the wall thickness of the triangles, an expensive analysis which is not run
     * after every edit.
     * @param trafo Transformation of the mesh to the units the thickness is measured in, the world
     * transformation of a part.
     */
    void analyze_thickness(const Domain::Transform3d& trafo);
    // Forget the thickness, for example after the scale of the mesh changed.
    void clear_thickness();

private:
    // Keep the marking only if the triangle indices did not change, new triangles stay unmarked.
    void update_after_edit(bool keep_marks);
    void update_marked_count();

    indexed_triangle_set m_mesh;
    std::unique_ptr<MeshTopology> m_topology;
    MeshDiagnostics m_diagnostics;
    std::vector<Hole> m_holes;
    FaceMask m_marked;
    size_t m_marked_count{0};
    FaceMask m_flipped;
    FaceMask m_noise;
    FaceMask m_intersecting;
    bool m_intersections_detected{false};
    std::vector<float> m_thickness;
    bool m_thickness_analyzed{false};
};

} // namespace Slic3r::Biz::Algorithms::MeshFix

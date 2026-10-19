#pragma once

#include <admesh/stl.h>

#include <cstddef>

#include "Slic3r/Biz/Algorithms/MeshFix/HoleFill.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/Marking.hpp"

/**
 * @brief Automatic repair operations working on the whole mesh.
 */
namespace Slic3r::Biz::Algorithms::MeshFix {

/**
 * @brief Make the orientation of triangles consistent and closed shells point outwards.
 * @return Number of flipped triangles.
 * @see mark_flipped_faces()
 */
size_t unify_normals(indexed_triangle_set& its);

/**
 * @brief Remove triangles referencing the same three vertices as another triangle.
 *
 * Pairs of oppositely oriented duplicates (zero thickness walls, typically inner walls
 * of merged parts) are removed completely, of the remaining equally oriented duplicates
 * a single triangle is kept.
 * @return Number of removed triangles.
 */
size_t remove_duplicate_faces(indexed_triangle_set& its);

/**
 * @brief Close cracks by merging vertices of open edges closer than the tolerance.
 *
 * Triangles collapsed by the merge and duplicate triangles are removed.
 * @return Number of merged vertices.
 */
size_t stitch_open_edges(indexed_triangle_set& its, float tolerance);

/**
 * @brief Remove shells marked by mark_noise_shells().
 * @return Number of removed triangles.
 */
size_t remove_noise_shells(indexed_triangle_set& its, const NoiseShellParams& params = {});

struct AutoFixParams
{
    // Open edge vertices closer than the tolerance are merged, in mesh units (mm).
    float stitch_tolerance{0.01f};
    bool remove_noise_shells{true};
    NoiseShellParams noise_shells;
    bool fill_holes{true};
    HoleFillParams hole_fill;
};

struct AutoFixReport
{
    size_t degenerate_faces_removed{0};
    size_t duplicate_faces_removed{0};
    size_t vertices_stitched{0};
    size_t faces_flipped{0};
    size_t noise_faces_removed{0};
    size_t holes_filled{0};
    size_t faces_added{0};

    bool changed() const
    {
        return degenerate_faces_removed > 0
            || duplicate_faces_removed > 0
            || vertices_stitched > 0
            || faces_flipped > 0
            || noise_faces_removed > 0
            || holes_filled > 0;
    }
};

/**
 * @brief Run the automatic repair operations in the order, in which they do not spoil each other:
 * degenerate triangles, duplicate triangles, stitching, normals, noise shells and holes.
 */
AutoFixReport auto_fix(indexed_triangle_set& its, const AutoFixParams& params = {});

} // namespace Slic3r::Biz::Algorithms::MeshFix

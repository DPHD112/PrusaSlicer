#pragma once

#include <admesh/stl.h>

#include <cstddef>
#include <vector>

#include "Slic3r/Biz/Algorithms/MeshFix/MeshTopology.hpp"

/**
 * @brief Marking (selection) of mesh triangles for manual mesh fixing.
 *
 * All functions return a FaceMask with one flag per triangle of the mesh, so that the results
 * can be combined with an existing marking (added, removed, intersected).
 */
namespace Slic3r::Biz::Algorithms::MeshFix {

/**
 * @brief Connected components of a mesh, triangles being connected over shared edges.
 */
struct Shells
{
    // Shell index of each triangle.
    std::vector<int> face_shell;
    // Number of triangles of each shell.
    std::vector<size_t> shell_sizes;

    size_t count() const
    {
        return shell_sizes.size();
    }
};

Shells find_shells(const MeshTopology& topology);

/**
 * @brief Mark all triangles connected to the seed triangle over shared edges.
 */
FaceMask mark_shell(const MeshTopology& topology, int seed_face);

/**
 * @brief Mark a smooth surface around the seed triangle.
 *
 * Grows the marking over edges shared by two triangles as long as the angle between the normals
 * of the neighbors does not exceed the tolerance, so the marking stops at sharp edges.
 * A curved surface of a fine mesh (cylinder, fillet) is marked as a whole.
 */
FaceMask mark_surface(
    const indexed_triangle_set& its,
    const MeshTopology& topology,
    int seed_face,
    float angle_tolerance_deg
);

/**
 * @brief Mark a planar region around the seed triangle.
 *
 * Grows the marking over edges shared by two triangles to triangles, whose normal deviates
 * from the normal of the seed triangle at most by the angle tolerance and whose vertices are
 * at most distance_tolerance away from the plane of the seed triangle.
 */
FaceMask mark_plane(
    const indexed_triangle_set& its,
    const MeshTopology& topology,
    int seed_face,
    float angle_tolerance_deg,
    float distance_tolerance
);

/**
 * @brief Add triangles sharing a vertex with a marked triangle.
 */
FaceMask grow_marking(const indexed_triangle_set& its, const FaceMask& mask);

/**
 * @brief Remove marked triangles sharing a vertex with an unmarked triangle.
 */
FaceMask shrink_marking(const indexed_triangle_set& its, const FaceMask& mask);

/**
 * @brief Mark triangles with an open or non-manifold edge.
 */
FaceMask mark_bad_edge_faces(const MeshTopology& topology);

/**
 * @brief Mark triangles whose orientation disagrees with the rest of their shell.
 *
 * The orientation is propagated over edges shared by exactly two triangles. Each consistently
 * connected region is then oriented to have a positive volume. Regions with a negligible volume
 * (open flat surfaces) keep the orientation of the majority of their triangles, and so do closed
 * regions inside of another closed region: those bound a cavity facing inwards or a body embedded
 * in the solid facing outwards.
 * Flipping the marked triangles makes the mesh consistently oriented, see unify_normals().
 */
FaceMask mark_flipped_faces(const indexed_triangle_set& its, const MeshTopology& topology);

struct NoiseShellParams
{
    // Shells with at most this number of triangles are noise.
    size_t max_faces{10};
    // Shells with bounding box diagonal at most this fraction of the mesh bounding box diagonal are noise.
    float max_relative_size{0.01f};
};

/**
 * @brief Mark small shells, typically leftovers of a bad export or scan.
 *
 * The largest shell of the mesh is never considered noise.
 */
FaceMask mark_noise_shells(
    const indexed_triangle_set& its,
    const MeshTopology& topology,
    const NoiseShellParams& params = {}
);

} // namespace Slic3r::Biz::Algorithms::MeshFix

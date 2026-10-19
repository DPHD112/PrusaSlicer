#pragma once

#include <admesh/stl.h>

#include <cstddef>

#include "Slic3r/Biz/Algorithms/MeshFix/Marking.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/MeshTopology.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix {

/**
 * @brief Summary of mesh errors, as reported before fixing a mesh.
 */
struct MeshDiagnostics
{
    size_t faces{0};
    size_t vertices{0};
    // Triangles with repeated vertices or with zero area.
    size_t degenerate_faces{0};
    // Edges used by a single triangle ("bad edges").
    size_t open_edges{0};
    // Edges shared by more than two triangles.
    size_t non_manifold_edges{0};
    // Edges between neighbors with opposite orientation.
    size_t inconsistent_edges{0};
    // Triangles to flip to get consistent outward orientation, see mark_flipped_faces().
    size_t flipped_faces{0};
    // Closed loops of open edges, see find_holes().
    size_t holes{0};
    size_t shells{0};
    size_t noise_shells{0};
    // Triangles intersecting another triangle, only evaluated when requested by DiagnoseParams.
    size_t intersecting_faces{0};

    bool is_watertight() const
    {
        return open_edges == 0 && non_manifold_edges == 0;
    }

    bool is_ok() const
    {
        return is_watertight()
            && inconsistent_edges == 0
            && flipped_faces == 0
            && degenerate_faces == 0
            && noise_shells == 0
            && intersecting_faces == 0;
    }
};

struct DiagnoseParams
{
    NoiseShellParams noise_shells;
    // Detection of intersecting triangles is the most expensive check.
    bool find_intersections{false};
};

MeshDiagnostics diagnose(const indexed_triangle_set& its, const DiagnoseParams& params = {});

/**
 * @brief Mark triangles with repeated vertices or with zero area.
 */
FaceMask mark_degenerate_faces(const indexed_triangle_set& its);

/**
 * @brief Mark triangles crossing another triangle of the mesh.
 *
 * Triangles sharing an edge and coplanar overlaps are not reported.
 */
FaceMask mark_intersecting_faces(const indexed_triangle_set& its);

} // namespace Slic3r::Biz::Algorithms::MeshFix

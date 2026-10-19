#pragma once

#include <admesh/stl.h>

#include <cstddef>
#include <vector>

#include "Slic3r/Biz/Algorithms/MeshFix/MeshTopology.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix {

/**
 * @brief Closed loop of open edges bounding a hole in a mesh.
 */
struct Hole
{
    // Boundary vertices ordered so that the triangles filling the hole traverse the loop in this order:
    // the existing triangle along the side vertices[i] -> vertices[i + 1] traverses it in the opposite direction.
    std::vector<int> vertices;
    // Existing triangle along the side vertices[i] -> vertices[(i + 1) % size()].
    std::vector<int> faces;

    size_t size() const
    {
        return vertices.size();
    }
};

/**
 * @brief Find closed loops of open edges.
 *
 * A boundary touching itself in a single vertex is split into simple loops. Open edges, which
 * do not form a closed loop (typically next to flipped triangles), are ignored, so it is better
 * to unify the normals before looking for holes.
 */
std::vector<Hole> find_holes(const indexed_triangle_set& its, const MeshTopology& topology);

double hole_perimeter(const indexed_triangle_set& its, const Hole& hole);

/**
 * @brief Index of the hole with a side between the two vertices, -1 if there is no such hole.
 */
int find_hole_by_edge(const std::vector<Hole>& holes, int vertex_a, int vertex_b);

enum class HoleFillMode
{
    // Triangulate the hole without adding vertices.
    Triangulate,
    // Add vertices so that the patch triangles are of similar size to the triangles around the hole
    // and fair the patch as a membrane. A planar hole gets a planar patch.
    Flat,
    // As Flat, but the patch continues the curvature of the surface around the hole.
    Smooth
};

struct HoleFillParams
{
    HoleFillMode mode{HoleFillMode::Smooth};
    // Holes with more vertices are triangulated by a faster but lower quality greedy algorithm,
    // as the optimal triangulation takes O(n^3) time and O(n^2) memory.
    size_t max_optimal_triangulation_size{500};
};

struct HoleFillResult
{
    size_t holes_filled{0};
    // Indices of the added triangles.
    std::vector<int> new_faces;
};

/**
 * @brief Fill holes by patches of new triangles connected to the hole boundaries.
 *
 * New vertices and triangles are appended to the mesh, the existing ones are not modified.
 * The holes have to be found on the mesh before any triangles were removed from it.
 */
HoleFillResult fill_holes(
    indexed_triangle_set& its,
    const std::vector<Hole>& holes,
    const HoleFillParams& params = {}
);

/**
 * @brief Fill a single hole, see fill_holes().
 * @return Indices of the added triangles, empty if the hole could not be filled.
 */
std::vector<int>
fill_hole(indexed_triangle_set& its, const Hole& hole, const HoleFillParams& params = {});

} // namespace Slic3r::Biz::Algorithms::MeshFix

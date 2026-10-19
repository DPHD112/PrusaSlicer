#pragma once

#include <admesh/stl.h>

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "Slic3r/Domain/Types.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix {

/**
 * @brief Per triangle flag, used to mark a subset of mesh triangles.
 */
using FaceMask = std::vector<bool>;

/**
 * @brief Classification of an undirected mesh edge by the triangles using it.
 */
enum class EdgeKind : uint8_t
{
    // Used by a single triangle, the edge lies on the border of a hole ("bad edge").
    Open,
    // Shared by two triangles with consistent orientation.
    Manifold,
    // Shared by two triangles traversing it in the same direction, one of them is flipped.
    Inconsistent,
    // Shared by more than two triangles.
    NonManifold
};

/**
 * @brief Side of a triangle, the side `edge` goes from vertex `edge` to vertex `(edge + 1) % 3`.
 */
struct FaceEdge
{
    int face;
    int edge;
};

/**
 * @brief Edge adjacency of an indexed triangle set, tolerant to open, flipped and non-manifold geometry.
 *
 * Unlike its_face_neighbors(), the topology keeps all triangles incident with an edge, so that
 * non-manifold edges and inconsistently oriented neighbors can be reported and repaired.
 * The topology refers to the vertex and triangle indices of the mesh, it has to be rebuilt
 * after the mesh is modified.
 */
class MeshTopology
{
public:
    explicit MeshTopology(const indexed_triangle_set& its);

    size_t faces_count() const
    {
        return m_face_edges.size();
    }

    size_t edges_count() const
    {
        return m_edge_vertices.size();
    }

    /**
     * @return Index of the edge on the given triangle side,
     * -1 if the side connects a vertex to itself (degenerate triangle).
     */
    int edge_index(int face, int edge) const
    {
        return m_face_edges[face][edge];
    }

    /**
     * @return Vertex indices of an edge, the smaller one first.
     */
    const Domain::Index2& edge_vertices(int edge_index) const
    {
        return m_edge_vertices[edge_index];
    }

    /**
     * @return Triangle sides incident with an edge.
     */
    std::span<const FaceEdge> edge_faces(int edge_index) const;

    EdgeKind edge_kind(int edge_index) const
    {
        return m_edge_kinds[edge_index];
    }

    /**
     * @return Index of the edge connecting the two vertices, -1 if there is no such edge.
     */
    int find_edge(int vertex_a, int vertex_b) const;

    /**
     * @brief Neighbor triangle across a triangle side.
     * @return Index of the other triangle if the edge is shared by exactly two triangles, -1 otherwise.
     */
    int neighbor(int face, int edge) const;

    /**
     * @brief Number of edges of the given kind.
     */
    size_t count(EdgeKind kind) const;

private:
    std::vector<Domain::Index3> m_face_edges;
    // Sorted lexicographically, which allows for binary search in find_edge().
    std::vector<Domain::Index2> m_edge_vertices;
    std::vector<EdgeKind> m_edge_kinds;
    // m_edge_faces[m_edge_faces_start[i]] .. m_edge_faces[m_edge_faces_start[i + 1]] are sides of edge i.
    std::vector<size_t> m_edge_faces_start;
    std::vector<FaceEdge> m_edge_faces;
};

/**
 * @brief Indices of the marked triangles.
 */
std::vector<int> mask_to_indices(const FaceMask& mask);

} // namespace Slic3r::Biz::Algorithms::MeshFix

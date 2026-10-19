#pragma once

#include <admesh/stl.h>

#include <algorithm>
#include <vector>

#include "Slic3r/Biz/Algorithms/MeshFix/MeshTopology.hpp"
#include "Slic3r/Domain/Types.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix::Test {

inline double signed_volume(const indexed_triangle_set& its)
{
    double volume = 0.;
    for (const stl_triangle_vertex_indices& face : its.indices) {
        const Domain::Vec3d v0 = its.vertices[face[0]].cast<double>();
        const Domain::Vec3d v1 = its.vertices[face[1]].cast<double>();
        const Domain::Vec3d v2 = its.vertices[face[2]].cast<double>();
        volume += v0.dot(v1.cross(v2)) / 6.;
    }
    return volume;
}

inline bool is_closed_and_oriented(const indexed_triangle_set& its)
{
    const MeshTopology topology(its);
    return topology.count(EdgeKind::Open) == 0
        && topology.count(EdgeKind::NonManifold) == 0
        && topology.count(EdgeKind::Inconsistent) == 0;
}

inline size_t count_marked(const FaceMask& mask)
{
    return size_t(std::count(mask.begin(), mask.end(), true));
}

/**
 * Remove the triangles matching the predicate, keeping the vertices.
 */
template <typename Predicate>
void remove_faces_if(indexed_triangle_set& its, Predicate predicate)
{
    std::vector<stl_triangle_vertex_indices> kept;
    for (size_t face = 0; face < its.indices.size(); ++face)
        if (!predicate(face))
            kept.push_back(its.indices[face]);
    its.indices = std::move(kept);
}

inline Domain::Vec3f centroid(const indexed_triangle_set& its, size_t face)
{
    const stl_triangle_vertex_indices& indices = its.indices[face];
    return (its.vertices[indices[0]] + its.vertices[indices[1]] + its.vertices[indices[2]]) / 3.f;
}

/**
 * Append a mesh, its vertices are not merged with the existing ones.
 */
inline void
append(indexed_triangle_set& its, const indexed_triangle_set& other, const Domain::Vec3f& offset)
{
    const int first_vertex = int(its.vertices.size());
    for (const stl_vertex& vertex : other.vertices)
        its.vertices.push_back(vertex + offset);
    for (const stl_triangle_vertex_indices& face : other.indices)
        its.indices.push_back(
            {face[0] + first_vertex, face[1] + first_vertex, face[2] + first_vertex}
        );
}

} // namespace Slic3r::Biz::Algorithms::MeshFix::Test

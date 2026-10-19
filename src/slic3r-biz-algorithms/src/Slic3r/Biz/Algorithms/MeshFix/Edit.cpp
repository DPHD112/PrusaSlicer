#include "Slic3r/Biz/Algorithms/MeshFix/Edit.hpp"

#include <algorithm>
#include <utility>

#include "Slic3r/Biz/Algorithms/TriangleMesh.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix {

using Biz::Algorithms::TriangleMesh::its_compactify_vertices;

size_t delete_faces(indexed_triangle_set& its, const FaceMask& mask)
{
    size_t kept = 0;
    for (size_t face = 0; face < its.indices.size(); ++face)
        if (!mask[face])
            its.indices[kept++] = its.indices[face];
    const size_t removed = its.indices.size() - kept;
    its.indices.resize(kept);
    if (removed > 0)
        its_compactify_vertices(its);
    return removed;
}

size_t flip_faces(indexed_triangle_set& its, const FaceMask& mask)
{
    size_t flipped = 0;
    for (size_t face = 0; face < its.indices.size(); ++face)
        if (mask[face]) {
            std::swap(its.indices[face][1], its.indices[face][2]);
            ++flipped;
        }
    return flipped;
}

indexed_triangle_set extract_faces(indexed_triangle_set& its, const FaceMask& mask)
{
    indexed_triangle_set extracted;
    extracted.vertices = its.vertices;
    for (size_t face = 0; face < its.indices.size(); ++face)
        if (mask[face])
            extracted.indices.push_back(its.indices[face]);
    its_compactify_vertices(extracted);
    delete_faces(its, mask);
    return extracted;
}

} // namespace Slic3r::Biz::Algorithms::MeshFix

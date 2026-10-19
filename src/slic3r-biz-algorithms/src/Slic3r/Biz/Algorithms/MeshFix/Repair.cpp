#include "Slic3r/Biz/Algorithms/MeshFix/Repair.hpp"

#include <algorithm>
#include <limits>
#include <numeric>
#include <tuple>

#include "Slic3r/Biz/Algorithms/KDTreeIndirect.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/Edit.hpp"
#include "Slic3r/Biz/Algorithms/TriangleMesh.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix {

using Biz::Algorithms::TriangleMesh::its_compactify_vertices;
using Biz::Algorithms::TriangleMesh::its_remove_degenerate_faces;
using Domain::Index3;

size_t unify_normals(indexed_triangle_set& its)
{
    const MeshTopology topology(its);
    return flip_faces(its, mark_flipped_faces(its, topology));
}

size_t remove_duplicate_faces(indexed_triangle_set& its)
{
    struct Entry
    {
        Index3 sorted;
        // Orientation of the triangle relative to the sorted vertex order.
        bool positive;
        int face;
    };

    std::vector<Entry> entries;
    entries.reserve(its.indices.size());
    for (size_t face = 0; face < its.indices.size(); ++face) {
        const stl_triangle_vertex_indices& indices = its.indices[face];
        const int first = int(std::min_element(indices.begin(), indices.end()) - indices.begin());
        const bool positive = indices[(first + 1) % 3] < indices[(first + 2) % 3];
        Index3 sorted       = indices;
        std::sort(sorted.begin(), sorted.end());
        entries.push_back({sorted, positive, int(face)});
    }
    std::sort(
        entries.begin(),
        entries.end(),
        [](const Entry& lhs, const Entry& rhs)
        { return std::tie(lhs.sorted, lhs.face) < std::tie(rhs.sorted, rhs.face); }
    );

    FaceMask remove(its.indices.size(), false);
    for (size_t begin = 0; begin < entries.size();) {
        size_t end = begin + 1;
        while (end < entries.size() && entries[end].sorted == entries[begin].sorted)
            ++end;
        if (end - begin > 1) {
            const size_t positive_count = size_t(
                std::count_if(
                    entries.begin() + begin,
                    entries.begin() + end,
                    [](const Entry& entry) { return entry.positive; }
                )
            );
            const size_t negative_count = end - begin - positive_count;
            // Oppositely oriented duplicates cancel out, one of the remaining ones is kept.
            const bool keep_positive = positive_count > negative_count;
            bool keep_one            = positive_count != negative_count;
            for (size_t i = begin; i < end; ++i) {
                if (keep_one && entries[i].positive == keep_positive)
                    keep_one = false;
                else
                    remove[entries[i].face] = true;
            }
        }
        begin = end;
    }
    return delete_faces(its, remove);
}

size_t stitch_open_edges(indexed_triangle_set& its, float tolerance)
{
    if (tolerance <= 0.f)
        return 0;

    std::vector<int> open_vertices;
    {
        const MeshTopology topology(its);
        std::vector<bool> is_open(its.vertices.size(), false);
        for (size_t edge_idx = 0; edge_idx < topology.edges_count(); ++edge_idx)
            if (topology.edge_kind(int(edge_idx)) == EdgeKind::Open)
                for (int vertex_idx : topology.edge_vertices(int(edge_idx)))
                    if (!is_open[vertex_idx]) {
                        is_open[vertex_idx] = true;
                        open_vertices.push_back(vertex_idx);
                    }
    }
    if (open_vertices.size() < 2)
        return 0;

    auto coordinate = [&its, &open_vertices](size_t idx, size_t dimension)
    { return its.vertices[open_vertices[idx]][dimension]; };
    const KDTreeIndirect::KDTreeIndirect<3, float, decltype(coordinate)>
        tree(coordinate, open_vertices.size());

    // Union find over the open vertices.
    std::vector<size_t> parent(open_vertices.size());
    std::iota(parent.begin(), parent.end(), size_t(0));
    auto find_root = [&parent](size_t idx)
    {
        while (parent[idx] != idx) {
            parent[idx] = parent[parent[idx]];
            idx         = parent[idx];
        }
        return idx;
    };
    for (size_t i = 0; i < open_vertices.size(); ++i)
        for (size_t j :
             KDTreeIndirect::find_nearby_points(tree, its.vertices[open_vertices[i]], tolerance))
        {
            const size_t root_i = find_root(i);
            const size_t root_j = find_root(j);
            if (root_i != root_j)
                parent[std::max(root_i, root_j)] = std::min(root_i, root_j);
        }

    // Merge each group into its vertex with the lowest index.
    std::vector<int> representative(open_vertices.size(), std::numeric_limits<int>::max());
    for (size_t i = 0; i < open_vertices.size(); ++i) {
        int& group_vertex = representative[find_root(i)];
        group_vertex      = std::min(group_vertex, open_vertices[i]);
    }
    std::vector<int> vertex_map(its.vertices.size());
    std::iota(vertex_map.begin(), vertex_map.end(), 0);
    size_t merged = 0;
    for (size_t i = 0; i < open_vertices.size(); ++i) {
        const int group_vertex = representative[find_root(i)];
        if (group_vertex != open_vertices[i]) {
            vertex_map[open_vertices[i]] = group_vertex;
            ++merged;
        }
    }
    if (merged == 0)
        return 0;

    for (stl_triangle_vertex_indices& indices : its.indices)
        for (int& vertex_idx : indices)
            vertex_idx = vertex_map[vertex_idx];
    its_remove_degenerate_faces(its);
    remove_duplicate_faces(its);
    its_compactify_vertices(its);
    return merged;
}

size_t remove_noise_shells(indexed_triangle_set& its, const NoiseShellParams& params)
{
    const MeshTopology topology(its);
    return delete_faces(its, mark_noise_shells(its, topology, params));
}

AutoFixReport auto_fix(indexed_triangle_set& its, const AutoFixParams& params)
{
    AutoFixReport report;
    report.degenerate_faces_removed = size_t(its_remove_degenerate_faces(its));
    // Before stitching, which removes the duplicates it creates itself.
    report.duplicate_faces_removed = remove_duplicate_faces(its);
    report.vertices_stitched       = stitch_open_edges(its, params.stitch_tolerance);
    report.faces_flipped           = unify_normals(its);
    if (params.remove_noise_shells)
        report.noise_faces_removed = remove_noise_shells(its, params.noise_shells);
    if (params.fill_holes) {
        const MeshTopology topology(its);
        const HoleFillResult filled = fill_holes(its, find_holes(its, topology), params.hole_fill);
        report.holes_filled         = filled.holes_filled;
        report.faces_added          = filled.new_faces.size();
    }
    its_compactify_vertices(its);
    return report;
}

} // namespace Slic3r::Biz::Algorithms::MeshFix

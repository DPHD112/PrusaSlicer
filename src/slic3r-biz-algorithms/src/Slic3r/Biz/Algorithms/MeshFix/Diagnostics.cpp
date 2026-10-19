#include "Slic3r/Biz/Algorithms/MeshFix/Diagnostics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "Slic3r/Biz/Algorithms/AABBTreeIndirect.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/HoleFill.hpp"
#include "Slic3r/Domain/Types.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix {

using Domain::Vec3d;

namespace {

size_t count_marked(const FaceMask& mask)
{
    return size_t(std::count(mask.begin(), mask.end(), true));
}

/**
 * Does the segment cross the interior of the triangle? Segments parallel with the triangle
 * and touching contacts are not reported (Moller-Trumbore).
 */
bool
segment_crosses_triangle(const Vec3d& p0, const Vec3d& p1, const std::array<Vec3d, 3>& triangle)
{
    constexpr double epsilon = 1e-9;
    const Vec3d direction    = p1 - p0;
    const Vec3d edge1        = triangle[1] - triangle[0];
    const Vec3d edge2        = triangle[2] - triangle[0];
    const Vec3d h            = direction.cross(edge2);
    const double determinant = edge1.dot(h);
    if (std::abs(determinant) <= epsilon * edge1.norm() * edge2.norm() * direction.norm())
        return false;
    const Vec3d s  = p0 - triangle[0];
    const double u = s.dot(h) / determinant;
    if (u <= epsilon || u >= 1. - epsilon)
        return false;
    const Vec3d q  = s.cross(edge1);
    const double v = direction.dot(q) / determinant;
    if (v <= epsilon || u + v >= 1. - epsilon)
        return false;
    const double t = edge2.dot(q) / determinant;
    return t > epsilon && t < 1. - epsilon;
}

std::array<Vec3d, 3> triangle_points(const indexed_triangle_set& its, int face)
{
    const stl_triangle_vertex_indices& indices = its.indices[face];
    return {
        its.vertices[indices[0]].cast<double>(),
        its.vertices[indices[1]].cast<double>(),
        its.vertices[indices[2]].cast<double>()
    };
}

bool faces_intersect(const indexed_triangle_set& its, int face_a, int face_b)
{
    const stl_triangle_vertex_indices& indices_a = its.indices[face_a];
    const stl_triangle_vertex_indices& indices_b = its.indices[face_b];
    int shared_count                             = 0;
    int shared_a                                 = -1;
    int shared_b                                 = -1;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (indices_a[i] == indices_b[j]) {
                ++shared_count;
                shared_a = i;
                shared_b = j;
            }
    // Neighbors over an edge and duplicate triangles are reported by the topology checks.
    if (shared_count > 1)
        return false;

    const std::array<Vec3d, 3> triangle_a = triangle_points(its, face_a);
    const std::array<Vec3d, 3> triangle_b = triangle_points(its, face_b);
    if (shared_count == 1) {
        // Triangles sharing a vertex intersect elsewhere only if the side opposite
        // to the shared vertex of one of them crosses the other triangle.
        return segment_crosses_triangle(
                   triangle_a[(shared_a + 1) % 3],
                   triangle_a[(shared_a + 2) % 3],
                   triangle_b
               )
            || segment_crosses_triangle(
                   triangle_b[(shared_b + 1) % 3],
                   triangle_b[(shared_b + 2) % 3],
                   triangle_a
            );
    }
    // Two non-coplanar triangles intersect if and only if a side of one of them crosses the other one.
    for (int i = 0; i < 3; ++i) {
        if (segment_crosses_triangle(triangle_a[i], triangle_a[(i + 1) % 3], triangle_b)
            || segment_crosses_triangle(triangle_b[i], triangle_b[(i + 1) % 3], triangle_a))
            return true;
    }
    return false;
}

} // namespace

FaceMask mark_degenerate_faces(const indexed_triangle_set& its)
{
    FaceMask mask(its.indices.size(), false);
    for (size_t face = 0; face < its.indices.size(); ++face) {
        const stl_triangle_vertex_indices& indices = its.indices[face];
        if (indices[0] == indices[1] || indices[1] == indices[2] || indices[2] == indices[0]) {
            mask[face] = true;
            continue;
        }
        const std::array<Vec3d, 3> triangle = triangle_points(its, int(face));
        const double longest_squared        = std::max(
            {(triangle[1] - triangle[0]).squaredNorm(),
                    (triangle[2] - triangle[1]).squaredNorm(),
                    (triangle[0] - triangle[2]).squaredNorm()}
        );
        const double doubled_area =
            (triangle[1] - triangle[0]).cross(triangle[2] - triangle[0]).norm();
        mask[face] = doubled_area <= 1e-10 * longest_squared;
    }
    return mask;
}

FaceMask mark_intersecting_faces(const indexed_triangle_set& its)
{
    FaceMask mask(its.indices.size(), false);
    if (its.indices.empty())
        return mask;

    const auto tree =
        AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(its.vertices, its.indices);
    using BoundingBox = Eigen::AlignedBox<float, 3>;
    for (size_t face = 0; face < its.indices.size(); ++face) {
        const stl_triangle_vertex_indices& indices = its.indices[face];
        BoundingBox bbox(its.vertices[indices[0]], its.vertices[indices[0]]);
        bbox.extend(its.vertices[indices[1]]);
        bbox.extend(its.vertices[indices[2]]);
        AABBTreeIndirect::traverse(
            tree,
            AABBTreeIndirect::intersecting(bbox),
            [&](const auto& node)
            {
                const int other = int(node.idx);
                if (size_t(other) > face && faces_intersect(its, int(face), other)) {
                    mask[face]  = true;
                    mask[other] = true;
                }
                return true;
            }
        );
    }
    return mask;
}

MeshDiagnostics diagnose(const indexed_triangle_set& its, const DiagnoseParams& params)
{
    MeshDiagnostics diagnostics;
    diagnostics.faces            = its.indices.size();
    diagnostics.vertices         = its.vertices.size();
    diagnostics.degenerate_faces = count_marked(mark_degenerate_faces(its));

    const MeshTopology topology(its);
    diagnostics.open_edges         = topology.count(EdgeKind::Open);
    diagnostics.non_manifold_edges = topology.count(EdgeKind::NonManifold);
    diagnostics.inconsistent_edges = topology.count(EdgeKind::Inconsistent);
    diagnostics.flipped_faces      = count_marked(mark_flipped_faces(its, topology));
    diagnostics.holes              = find_holes(its, topology).size();

    const Shells shells  = find_shells(topology);
    diagnostics.shells   = shells.count();
    const FaceMask noise = mark_noise_shells(its, topology, params.noise_shells);
    std::vector<bool> noise_shell(shells.count(), false);
    for (size_t face = 0; face < noise.size(); ++face)
        if (noise[face])
            noise_shell[shells.face_shell[face]] = true;
    diagnostics.noise_shells = size_t(std::count(noise_shell.begin(), noise_shell.end(), true));

    if (params.find_intersections)
        diagnostics.intersecting_faces = count_marked(mark_intersecting_faces(its));
    return diagnostics;
}

} // namespace Slic3r::Biz::Algorithms::MeshFix

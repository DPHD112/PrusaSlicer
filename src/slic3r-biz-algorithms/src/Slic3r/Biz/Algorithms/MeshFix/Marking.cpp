#include "Slic3r/Biz/Algorithms/MeshFix/Marking.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>

#include "Slic3r/Domain/Types.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix {

using Domain::Vec3d;

namespace {

Vec3d face_normal(const indexed_triangle_set& its, int face)
{
    const stl_triangle_vertex_indices& indices = its.indices[face];
    const Vec3d v0                             = its.vertices[indices[0]].cast<double>();
    const Vec3d v1                             = its.vertices[indices[1]].cast<double>();
    const Vec3d v2                             = its.vertices[indices[2]].cast<double>();
    const Vec3d normal                         = (v1 - v0).cross(v2 - v0);
    const double length                        = normal.norm();
    return length > 0. ? Vec3d(normal / length) : Vec3d::Zero();
}

double cos_of_degrees(float angle_deg)
{
    return std::cos(double(angle_deg) * std::numbers::pi / 180.);
}

bool is_valid_face(const MeshTopology& topology, int face)
{
    return face >= 0 && size_t(face) < topology.faces_count();
}

/**
 * Flood fill over edges shared by exactly two triangles.
 * The accept predicate receives the triangle to add and its orientation relative to the seed
 * (-1 when it is flipped with respect to the seed, 1 otherwise).
 */
template <typename AcceptFn>
FaceMask flood_fill_two_sided(const MeshTopology& topology, int seed_face, AcceptFn accept)
{
    FaceMask mask(topology.faces_count(), false);
    if (!is_valid_face(topology, seed_face))
        return mask;

    std::vector<int8_t> orientation(topology.faces_count(), 0);
    std::vector<int> queue{seed_face};
    mask[seed_face]        = true;
    orientation[seed_face] = 1;
    while (!queue.empty()) {
        const int face = queue.back();
        queue.pop_back();
        for (int edge = 0; edge < 3; ++edge) {
            const int edge_idx = topology.edge_index(face, edge);
            if (edge_idx < 0)
                continue;
            const EdgeKind kind = topology.edge_kind(edge_idx);
            if (kind != EdgeKind::Manifold && kind != EdgeKind::Inconsistent)
                continue;
            const int other = topology.neighbor(face, edge);
            if (other < 0 || mask[other])
                continue;
            const int8_t other_orientation =
                kind == EdgeKind::Manifold ? orientation[face] : -orientation[face];
            if (accept(face, orientation[face], other, other_orientation)) {
                mask[other]        = true;
                orientation[other] = other_orientation;
                queue.push_back(other);
            }
        }
    }
    return mask;
}

} // namespace

Shells find_shells(const MeshTopology& topology)
{
    Shells shells;
    shells.face_shell.assign(topology.faces_count(), -1);
    std::vector<int> queue;
    for (size_t seed = 0; seed < topology.faces_count(); ++seed) {
        if (shells.face_shell[seed] >= 0)
            continue;
        const int shell_idx     = int(shells.shell_sizes.size());
        size_t shell_size       = 1;
        shells.face_shell[seed] = shell_idx;
        queue.assign(1, int(seed));
        while (!queue.empty()) {
            const int face = queue.back();
            queue.pop_back();
            for (int edge = 0; edge < 3; ++edge) {
                const int edge_idx = topology.edge_index(face, edge);
                if (edge_idx < 0)
                    continue;
                for (const FaceEdge& side : topology.edge_faces(edge_idx)) {
                    if (shells.face_shell[side.face] < 0) {
                        shells.face_shell[side.face] = shell_idx;
                        ++shell_size;
                        queue.push_back(side.face);
                    }
                }
            }
        }
        shells.shell_sizes.push_back(shell_size);
    }
    return shells;
}

FaceMask mark_shell(const MeshTopology& topology, int seed_face)
{
    FaceMask mask(topology.faces_count(), false);
    if (!is_valid_face(topology, seed_face))
        return mask;

    std::vector<int> queue{seed_face};
    mask[seed_face] = true;
    while (!queue.empty()) {
        const int face = queue.back();
        queue.pop_back();
        for (int edge = 0; edge < 3; ++edge) {
            const int edge_idx = topology.edge_index(face, edge);
            if (edge_idx < 0)
                continue;
            for (const FaceEdge& side : topology.edge_faces(edge_idx)) {
                if (!mask[side.face]) {
                    mask[side.face] = true;
                    queue.push_back(side.face);
                }
            }
        }
    }
    return mask;
}

FaceMask mark_surface(
    const indexed_triangle_set& its,
    const MeshTopology& topology,
    int seed_face,
    float angle_tolerance_deg
)
{
    const double min_cos = cos_of_degrees(angle_tolerance_deg);
    return flood_fill_two_sided(
        topology,
        seed_face,
        [&](int face, int8_t orientation, int other, int8_t other_orientation)
        {
            const Vec3d normal       = face_normal(its, face);
            const Vec3d other_normal = face_normal(its, other);
            // Degenerate triangles do not stop the marking.
            if (normal.isZero() || other_normal.isZero())
                return true;
            return double(orientation * other_orientation) * normal.dot(other_normal) >= min_cos;
        }
    );
}

FaceMask mark_plane(
    const indexed_triangle_set& its,
    const MeshTopology& topology,
    int seed_face,
    float angle_tolerance_deg,
    float distance_tolerance
)
{
    if (!is_valid_face(topology, seed_face))
        return FaceMask(topology.faces_count(), false);

    const double min_cos     = cos_of_degrees(angle_tolerance_deg);
    const Vec3d plane_normal = face_normal(its, seed_face);
    const Vec3d plane_point  = its.vertices[its.indices[seed_face][0]].cast<double>();
    return flood_fill_two_sided(
        topology,
        seed_face,
        [&](int, int8_t, int other, int8_t other_orientation)
        {
            const Vec3d other_normal = face_normal(its, other);
            if (plane_normal.isZero() || other_normal.isZero())
                return false;
            if (double(other_orientation) * plane_normal.dot(other_normal) < min_cos)
                return false;
            for (int vertex_idx : its.indices[other]) {
                const Vec3d vertex = its.vertices[vertex_idx].cast<double>();
                if (std::abs(plane_normal.dot(vertex - plane_point)) > distance_tolerance)
                    return false;
            }
            return true;
        }
    );
}

FaceMask grow_marking(const indexed_triangle_set& its, const FaceMask& mask)
{
    std::vector<bool> marked_vertex(its.vertices.size(), false);
    for (size_t face = 0; face < its.indices.size(); ++face)
        if (mask[face])
            for (int vertex_idx : its.indices[face])
                marked_vertex[vertex_idx] = true;

    FaceMask result(mask);
    for (size_t face = 0; face < its.indices.size(); ++face) {
        const stl_triangle_vertex_indices& indices = its.indices[face];
        if (marked_vertex[indices[0]] || marked_vertex[indices[1]] || marked_vertex[indices[2]])
            result[face] = true;
    }
    return result;
}

FaceMask shrink_marking(const indexed_triangle_set& its, const FaceMask& mask)
{
    std::vector<bool> unmarked_vertex(its.vertices.size(), false);
    for (size_t face = 0; face < its.indices.size(); ++face)
        if (!mask[face])
            for (int vertex_idx : its.indices[face])
                unmarked_vertex[vertex_idx] = true;

    FaceMask result(mask);
    for (size_t face = 0; face < its.indices.size(); ++face) {
        const stl_triangle_vertex_indices& indices = its.indices[face];
        if (unmarked_vertex[indices[0]]
            || unmarked_vertex[indices[1]]
            || unmarked_vertex[indices[2]])
            result[face] = false;
    }
    return result;
}

FaceMask mark_bad_edge_faces(const MeshTopology& topology)
{
    FaceMask mask(topology.faces_count(), false);
    for (size_t edge_idx = 0; edge_idx < topology.edges_count(); ++edge_idx) {
        const EdgeKind kind = topology.edge_kind(int(edge_idx));
        if (kind == EdgeKind::Open || kind == EdgeKind::NonManifold)
            for (const FaceEdge& side : topology.edge_faces(int(edge_idx)))
                mask[side.face] = true;
    }
    return mask;
}

FaceMask mark_flipped_faces(const indexed_triangle_set& its, const MeshTopology& topology)
{
    const size_t faces_count = topology.faces_count();
    FaceMask mask(faces_count, false);
    std::vector<int8_t> orientation(faces_count, 0);
    std::vector<int> region;

    for (size_t seed = 0; seed < faces_count; ++seed) {
        if (orientation[seed] != 0)
            continue;

        // Propagate the orientation of the seed over edges shared by two triangles.
        region.assign(1, int(seed));
        orientation[seed] = 1;
        for (size_t i = 0; i < region.size(); ++i) {
            const int face = region[i];
            for (int edge = 0; edge < 3; ++edge) {
                const int edge_idx = topology.edge_index(face, edge);
                if (edge_idx < 0)
                    continue;
                const EdgeKind kind = topology.edge_kind(edge_idx);
                if (kind != EdgeKind::Manifold && kind != EdgeKind::Inconsistent)
                    continue;
                const int other = topology.neighbor(face, edge);
                if (other >= 0 && orientation[other] == 0) {
                    orientation[other] =
                        kind == EdgeKind::Manifold ? orientation[face] : -orientation[face];
                    region.push_back(other);
                }
            }
        }

        // Signed volume of the region with the propagated orientation, measured relative
        // to the region centroid to limit the round off errors.
        Vec3d centroid = Vec3d::Zero();
        Eigen::AlignedBox<double, 3> bbox;
        for (int face : region)
            for (int vertex_idx : its.indices[face]) {
                const Vec3d vertex = its.vertices[vertex_idx].cast<double>();
                centroid += vertex;
                bbox.extend(vertex);
            }
        centroid /= double(3 * region.size());

        double volume         = 0.;
        size_t positive_count = 0;
        for (int face : region) {
            const stl_triangle_vertex_indices& indices = its.indices[face];
            const Vec3d v0 = its.vertices[indices[0]].cast<double>() - centroid;
            const Vec3d v1 = its.vertices[indices[1]].cast<double>() - centroid;
            const Vec3d v2 = its.vertices[indices[2]].cast<double>() - centroid;
            volume += double(orientation[face]) * v0.dot(v1.cross(v2)) / 6.;
            if (orientation[face] > 0)
                ++positive_count;
        }

        const double diagonal      = bbox.diagonal().norm();
        int8_t correct_orientation = 1;
        if (std::abs(volume) > 1e-6 * diagonal * diagonal * diagonal)
            correct_orientation = volume > 0. ? 1 : -1;
        else
            correct_orientation = 2 * positive_count >= region.size() ? 1 : -1;

        for (int face : region)
            if (orientation[face] != correct_orientation)
                mask[face] = true;
    }
    return mask;
}

FaceMask mark_noise_shells(
    const indexed_triangle_set& its,
    const MeshTopology& topology,
    const NoiseShellParams& params
)
{
    FaceMask mask(topology.faces_count(), false);
    const Shells shells = find_shells(topology);
    if (shells.count() < 2)
        return mask;

    std::vector<Eigen::AlignedBox<float, 3>> shell_boxes(shells.count());
    Eigen::AlignedBox<float, 3> mesh_box;
    for (size_t face = 0; face < topology.faces_count(); ++face)
        for (int vertex_idx : its.indices[face]) {
            shell_boxes[shells.face_shell[face]].extend(its.vertices[vertex_idx]);
            mesh_box.extend(its.vertices[vertex_idx]);
        }

    const size_t largest_shell = size_t(
        std::max_element(shells.shell_sizes.begin(), shells.shell_sizes.end())
        - shells.shell_sizes.begin()
    );
    const float max_diagonal = params.max_relative_size * mesh_box.diagonal().norm();
    std::vector<bool> is_noise(shells.count(), false);
    for (size_t shell_idx = 0; shell_idx < shells.count(); ++shell_idx)
        is_noise[shell_idx] = shell_idx != largest_shell
            && (shells.shell_sizes[shell_idx] <= params.max_faces
                || shell_boxes[shell_idx].diagonal().norm() <= max_diagonal);

    for (size_t face = 0; face < topology.faces_count(); ++face)
        mask[face] = is_noise[shells.face_shell[face]];
    return mask;
}

} // namespace Slic3r::Biz::Algorithms::MeshFix

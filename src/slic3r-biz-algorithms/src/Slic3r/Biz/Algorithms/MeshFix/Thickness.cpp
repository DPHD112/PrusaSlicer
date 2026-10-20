#include "Slic3r/Biz/Algorithms/MeshFix/Thickness.hpp"

#include <oneapi/tbb/blocked_range.h>
#include <oneapi/tbb/parallel_for.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "Slic3r/Biz/Algorithms/AABBTreeIndirect.hpp"
#include "Slic3r/Domain/Types.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix {

using Domain::Vec3d;

namespace {

constexpr float Unknown = std::numeric_limits<float>::infinity();

// Normal of the triangle facing out of the solid, not normalized.
Vec3d outward_normal(const indexed_triangle_set& its, const FaceMask& flipped, int face)
{
    const stl_triangle_vertex_indices& indices = its.indices[face];
    const Vec3d a                              = its.vertices[indices[0]].cast<double>();
    const Vec3d normal =
        (its.vertices[indices[1]].cast<double>() - a)
            .cross(its.vertices[indices[2]].cast<double>() - a);
    return !flipped.empty() && flipped[face] ? Vec3d(-normal) : normal;
}

// Distance along the ray to where it leaves the solid, infinity if it does not.
// The hits are reused between the calls to save allocations.
template <typename Tree>
double distance_to_exit(
    const indexed_triangle_set& its,
    const FaceMask& flipped,
    const Tree& tree,
    int face,
    const Vec3d& origin,
    const Vec3d& direction,
    double tolerance,
    std::vector<igl::Hit>& hits
)
{
    AABBTreeIndirect::
        intersect_ray_all_hits(its.vertices, its.indices, tree, origin, direction, hits);
    // Inside of the solid behind the triangle, each shell entered on the way has to be left.
    int depth = 1;
    for (size_t i = 0; i < hits.size();) {
        // Crossings at the same distance are one crossing: a ray through an edge or a vertex hits
        // all triangles around it, a ray from one shell into a touching one leaves and enters.
        const float distance = hits[i].t;
        bool enters          = false;
        bool leaves          = false;
        for (; i < hits.size() && hits[i].t - distance <= tolerance; ++i) {
            if (hits[i].id == face || hits[i].t <= tolerance)
                continue;
            const double facing = outward_normal(its, flipped, hits[i].id).dot(direction);
            enters              = enters || facing < 0.;
            leaves              = leaves || facing > 0.;
        }
        depth += int(enters) - int(leaves);
        if (depth <= 0)
            return double(distance);
    }
    return std::numeric_limits<double>::infinity();
}

} // namespace

std::vector<float> measure_thickness(const indexed_triangle_set& its, const FaceMask& flipped)
{
    std::vector<float> thickness(its.indices.size(), Unknown);
    if (its.indices.empty())
        return thickness;

    const auto tree =
        AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(its.vertices, its.indices);
    // Crossings closer than the tolerance are merged, it is far below the float precision of
    // the coordinates of a printable part.
    const Eigen::AlignedBox<float, 3>& box = tree.nodes().front().bbox;
    const double tolerance                 = 1e-6 * (box.max() - box.min()).cast<double>().norm();

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, its.indices.size()),
        [&](const tbb::blocked_range<size_t>& range)
        {
            std::vector<igl::Hit> hits;
            for (size_t face = range.begin(); face < range.end(); ++face) {
                const Vec3d normal = outward_normal(its, flipped, int(face));
                if (normal.squaredNorm() == 0.)
                    continue;
                const Vec3d direction = -normal.normalized();

                const stl_triangle_vertex_indices& indices = its.indices[face];
                const std::array<Vec3d, 3> corners{
                    its.vertices[indices[0]].cast<double>(),
                    its.vertices[indices[1]].cast<double>(),
                    its.vertices[indices[2]].cast<double>()
                };
                const Vec3d centroid = (corners[0] + corners[1] + corners[2]) / 3.;
                double shortest      = distance_to_exit(
                    its,
                    flipped,
                    tree,
                    int(face),
                    centroid,
                    direction,
                    tolerance,
                    hits
                );
                for (const Vec3d& corner : corners)
                    shortest = std::min(
                        shortest,
                        distance_to_exit(
                            its,
                            flipped,
                            tree,
                            int(face),
                            0.5 * (centroid + corner),
                            direction,
                            tolerance,
                            hits
                        )
                    );
                thickness[face] = float(shortest);
            }
        }
    );
    return thickness;
}

} // namespace Slic3r::Biz::Algorithms::MeshFix

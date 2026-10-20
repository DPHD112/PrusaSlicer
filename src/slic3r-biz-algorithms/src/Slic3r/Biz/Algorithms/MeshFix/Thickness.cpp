#include "Slic3r/Biz/Algorithms/MeshFix/Thickness.hpp"

#include <oneapi/tbb/blocked_range.h>
#include <oneapi/tbb/parallel_for.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include "Slic3r/Biz/Algorithms/AABBTreeIndirect.hpp"
#include "Slic3r/Domain/Types.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix {

using Domain::Vec3d;
using Domain::Vec3f;

namespace {

constexpr double Infinity = std::numeric_limits<double>::infinity();
// Distance returned for a point which is not on a wall.
constexpr double Inner = -1.;

using Tree = AABBTreeIndirect::Tree<3, float>;
using Box  = Tree::BoundingBox;

struct Hit
{
    double distance;
    int face;
};

// Memory reused between the rays of a thread.
struct Scratch
{
    std::vector<Hit> hits;
    // Other shells whose bounding box contains the point.
    std::vector<int> shells;
};

// Shells connected over edges of exactly two triangles, so that bodies touching along an edge are
// separate shells.
std::vector<int> manifold_shells(const MeshTopology& topology, size_t& count)
{
    std::vector<int> face_shell(topology.faces_count(), -1);
    std::vector<int> queue;
    count = 0;
    for (size_t seed = 0; seed < face_shell.size(); ++seed) {
        if (face_shell[seed] >= 0)
            continue;
        face_shell[seed] = int(count);
        queue.assign(1, int(seed));
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
                for (const FaceEdge& side : topology.edge_faces(edge_idx))
                    if (face_shell[side.face] < 0) {
                        face_shell[side.face] = int(count);
                        queue.push_back(side.face);
                    }
            }
        }
        ++count;
    }
    return face_shell;
}

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

class Measurement
{
public:
    Measurement(
        const indexed_triangle_set& its,
        const MeshTopology& topology,
        const FaceMask& flipped
    ) :
        m_its(its),
        m_flipped(flipped),
        m_tree(
            AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(its.vertices, its.indices)
        )
    {
        const Box& box        = m_tree.nodes().front().bbox;
        const double diagonal = (box.max() - box.min()).cast<double>().norm();
        const double magnitude =
            std::max(box.min().cwiseAbs().maxCoeff(), box.max().cwiseAbs().maxCoeff());
        // Crossings closer than the tolerance are merged. It is far below the size of a printable
        // detail, and above the rounding of the float coordinates.
        m_tolerance =
            std::max(1e-6 * diagonal, 16. * std::numeric_limits<float>::epsilon() * magnitude);
        // Triangles are hit unless the ray is parallel to them, also tiny ones of a tiny part.
        m_epsilon = 1e-12 * diagonal * diagonal;
        // Most walls are much thinner than the part, the ray is followed further only if needed.
        m_window = diagonal / 64.;

        size_t shells_count = 0;
        m_face_shell        = manifold_shells(topology, shells_count);
        // Only closed shells bound a solid: every edge is used by an even number of their
        // triangles. A closed shell of a negative volume bounds a cavity.
        m_shell_closed.assign(shells_count, true);
        m_shell_cavity.assign(shells_count, false);
        std::vector<double> volumes(shells_count, 0.);
        for (size_t face = 0; face < its.indices.size(); ++face) {
            const int shell = m_face_shell[face];
            for (int edge = 0; edge < 3; ++edge) {
                const int edge_idx = topology.edge_index(int(face), edge);
                if (edge_idx < 0)
                    continue;
                int same = 0;
                for (const FaceEdge& side : topology.edge_faces(edge_idx))
                    same += int(m_face_shell[side.face] == shell);
                if (same % 2 != 0)
                    m_shell_closed[shell] = false;
            }
            volumes[shell] +=
                its.vertices[its.indices[face][0]].cast<double>().dot(
                    outward_normal(its, flipped, int(face))
                )
                / 6.;
        }
        for (size_t shell = 0; shell < shells_count; ++shell)
            m_shell_cavity[shell] = m_shell_closed[shell] && volumes[shell] < 0.;

        if (shells_count > 1) {
            // Bounding boxes of the shells tell which points may lie inside of other shells.
            std::vector<Box> boxes(shells_count, Box());
            for (size_t face = 0; face < its.indices.size(); ++face)
                for (int vertex : its.indices[face])
                    boxes[m_face_shell[face]].extend(its.vertices[vertex]);

            struct Input
            {
                size_t idx() const
                {
                    return index;
                }

                const Box& bbox() const
                {
                    return box;
                }

                const Tree::VectorType& centroid() const
                {
                    return center;
                }

                size_t index;
                Box box;
                Tree::VectorType center;
            };

            std::vector<Input> input;
            input.reserve(boxes.size());
            const Vec3f margin = Vec3f::Constant(float(8. * m_tolerance));
            for (size_t shell = 0; shell < boxes.size(); ++shell) {
                boxes[shell] = Box(boxes[shell].min() - margin, boxes[shell].max() + margin);
                input.push_back({shell, boxes[shell], boxes[shell].center()});
            }
            m_shell_tree.build(std::move(input));
            m_shell_boxes = std::move(boxes);
        }
    }

    /**
     * Distance from the point of the triangle against its normal to where the ray leaves the
     * solid. Infinity if it does not leave the solid within the cap, Inner if the point lies inside
     * of other shells.
     */
    double thickness(
        int face,
        const Vec3d& point,
        const Vec3d& direction,
        double cap,
        Scratch& scratch
    ) const
    {
        const Vec3d inverse    = direction.cwiseInverse();
        std::vector<Hit>& hits = scratch.hits;

        // Inside of the shell of the triangle, each shell entered on the way has to be left.
        int depth = 1;
        if (other_shells_around(point, m_face_shell[face], scratch.shells)) {
            const std::optional<int> start =
                start_depth(point, direction, m_face_shell[face], scratch);
            if (!start.has_value())
                // Material of another shell just outside of the triangle makes the point an inner
                // one, it is not on a wall.
                return Inner;
            depth = *start;
        }

        // The nearest crossings decide, the ray is followed further only if needed.
        double from = 0.;
        double to   = std::min(m_window, cap);
        for (;;) {
            hits.clear();
            collect_hits(0, point, direction, inverse, from, to + 2. * m_tolerance, hits);
            std::sort(
                hits.begin(),
                hits.end(),
                [](const Hit& l, const Hit& r) { return l.distance < r.distance; }
            );
            for (size_t i = 0; i < hits.size() && hits[i].distance <= to;) {
                const double distance = hits[i].distance;
                depth -= crossing_exits(face, direction, hits, i);
                // The crossings up to here are done, the next ones are further.
                from = hits[i - 1].distance;
                if (depth <= 0)
                    return distance;
            }
            if (to >= cap)
                return Infinity;
            to = cap;
        }
    }

private:
    // Other closed shells whose bounding box contains the point, those may contain the point.
    bool other_shells_around(const Vec3d& point, int own_shell, std::vector<int>& shells) const
    {
        shells.clear();
        AABBTreeIndirect::traverse(
            m_shell_tree,
            AABBTreeIndirect::intersecting(Box(point.cast<float>(), point.cast<float>())),
            [this, &shells, own_shell](const Tree::Node& node)
            {
                if (int(node.idx) != own_shell && m_shell_closed[node.idx])
                    shells.push_back(int(node.idx));
                return true;
            }
        );
        return !shells.empty();
    }

    /**
     * Number of shells the point of the triangle is inside of, counting the shell of the triangle
     * and other shells continuing behind it, when the triangle lies on their surface. Empty if the
     * point lies inside of another shell, so that it is not on a wall.
     *
     * Follows the ray from just outside of the triangle through the bounding boxes of the other
     * shells around the point. The ray leaves each closed shell containing its start once more
     * than it enters it, and enters each cavity containing its start once more than it leaves it.
     * Their sum, the winding number, is positive inside of the solid.
     */
    std::optional<int>
    start_depth(const Vec3d& point, const Vec3d& direction, int own_shell, Scratch& scratch) const
    {
        const double offset    = 4. * m_tolerance;
        const Vec3d outside    = point - offset * direction;
        const Vec3d inverse    = direction.cwiseInverse();
        std::vector<Hit>& hits = scratch.hits;
        double end             = 0.;
        for (int shell : scratch.shells) {
            const Box& box = m_shell_boxes[shell];
            double exit    = Infinity;
            for (int axis = 0; axis < 3; ++axis) {
                const double near = (double(box.min()[axis]) - outside[axis]) * inverse[axis];
                const double far  = (double(box.max()[axis]) - outside[axis]) * inverse[axis];
                exit              = std::min(exit, std::max(near, far));
            }
            end = std::max(end, exit);
        }
        hits.clear();
        collect_hits(0, outside, direction, inverse, 0., end, hits);
        std::sort(
            hits.begin(),
            hits.end(),
            [](const Hit& l, const Hit& r) { return l.distance < r.distance; }
        );

        // Just outside of a cavity's own triangle the ray is inside of the cavity.
        int winding = m_shell_cavity[own_shell] ? -1 : 0;
        // The triangle itself is entered.
        int depth = 1;
        // The crossings the walk from the point does not count, as it skips them.
        const double skipped = offset + m_tolerance;
        for (int shell : scratch.shells) {
            bool enters_at = false;
            bool leaves_at = false;
            for (size_t i = 0; i < hits.size();) {
                // A ray through an edge or a vertex hits all triangles around it.
                const size_t begin = i;
                bool enters        = false;
                bool leaves        = false;
                for (; i < hits.size() && hits[i].distance - hits[begin].distance <= m_tolerance;
                     ++i)
                {
                    if (m_face_shell[hits[i].face] != shell)
                        continue;
                    const int kind = crossing(hits[i].face, direction);
                    enters         = enters || kind < 0;
                    leaves         = leaves || kind > 0;
                    if (hits[i].distance <= skipped) {
                        enters_at = enters_at || kind < 0;
                        leaves_at = leaves_at || kind > 0;
                    }
                }
                winding += int(leaves) - int(enters);
            }
            depth += int(enters_at) - int(leaves_at);
        }
        if (winding >= 1)
            return std::nullopt;
        return winding + depth;
    }

    // Hits of the ray with the triangles at distances in (from, to].
    void collect_hits(
        size_t node_idx,
        const Vec3d& origin,
        const Vec3d& direction,
        const Vec3d& inverse,
        double from,
        double to,
        std::vector<Hit>& hits
    ) const
    {
        const Tree::Node& node = m_tree.node(node_idx);
        if (!AABBTreeIndirect::detail::ray_box_intersect_invdir(
                origin,
                inverse,
                node.bbox.cast<double>(),
                std::max(0., from - m_tolerance),
                to + m_tolerance
            ))
            return;
        if (node.is_leaf()) {
            const stl_triangle_vertex_indices& indices = m_its.indices[node.idx];
            double distance                            = 0.;
            double u                                   = 0.;
            double v                                   = 0.;
            if (AABBTreeIndirect::detail::intersect_triangle(
                    origin,
                    direction,
                    m_its.vertices[indices[0]].cast<double>().eval(),
                    m_its.vertices[indices[1]].cast<double>().eval(),
                    m_its.vertices[indices[2]].cast<double>().eval(),
                    distance,
                    u,
                    v,
                    m_epsilon
                )
                && distance > from
                && distance <= to)
                hits.push_back({distance, int(node.idx)});
            return;
        }
        collect_hits(Tree::left_child_idx(node_idx), origin, direction, inverse, from, to, hits);
        collect_hits(Tree::right_child_idx(node_idx), origin, direction, inverse, from, to, hits);
    }

    // +1 when the ray leaves the shell of the triangle, -1 when it enters it, 0 when it grazes it.
    int crossing(int face, const Vec3d& direction) const
    {
        const double facing = outward_normal(m_its, m_flipped, face).dot(direction);
        return facing > 0. ? 1 : facing < 0. ? -1 : 0;
    }

    /**
     * Number of shells the ray leaves minus the number it enters at the hits from i on, which are
     * at the same distance: a ray through an edge or a vertex hits all triangles around it, they
     * count once per shell, a ray from a shell into a touching one leaves one and enters the
     * other. Moves i past these hits.
     */
    int
    crossing_exits(int face, const Vec3d& direction, const std::vector<Hit>& hits, size_t& i) const
    {
        const size_t begin = i;
        while (i < hits.size() && hits[i].distance - hits[begin].distance <= m_tolerance)
            ++i;
        // The own triangle and triangles touching the start of the ray are not crossed. Open
        // shells bound no solid, unless the triangle belongs to one, as an unwelded seam would.
        const bool closed = m_shell_closed[m_face_shell[face]];
        const auto counts = [&](size_t hit)
        {
            return hits[hit].face != face
                && hits[hit].distance > m_tolerance
                && (!closed || m_shell_closed[m_face_shell[hits[hit].face]]);
        };
        int exits = 0;
        for (size_t hit = begin; hit < i; ++hit) {
            if (!counts(hit))
                continue;
            const int kind  = crossing(hits[hit].face, direction);
            const int shell = m_face_shell[hits[hit].face];
            bool repeated   = kind == 0;
            for (size_t other = begin; other < hit && !repeated; ++other)
                repeated = counts(other)
                    && m_face_shell[hits[other].face] == shell
                    && crossing(hits[other].face, direction) == kind;
            if (!repeated)
                exits += kind;
        }
        return exits;
    }

    const indexed_triangle_set& m_its;
    const FaceMask& m_flipped;
    Tree m_tree;
    std::vector<int> m_face_shell;
    std::vector<bool> m_shell_closed;
    std::vector<bool> m_shell_cavity;
    // Bounding boxes of the shells, empty for a single shell.
    std::vector<Box> m_shell_boxes;
    Tree m_shell_tree;
    double m_tolerance{0.};
    double m_epsilon{0.};
    double m_window{0.};
};

} // namespace

std::vector<float> measure_thickness(
    const indexed_triangle_set& its,
    const MeshTopology& topology,
    const FaceMask& flipped
)
{
    std::vector<float> thickness(its.indices.size(), std::numeric_limits<float>::infinity());
    if (its.indices.empty())
        return thickness;

    const Measurement measurement(its, topology, flipped);
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, its.indices.size()),
        [&](const tbb::blocked_range<size_t>& range)
        {
            Scratch scratch;
            for (size_t face = range.begin(); face < range.end(); ++face) {
                const Vec3d normal = outward_normal(its, flipped, int(face));
                if (normal.squaredNorm() == 0.)
                    continue;
                const Vec3d direction = -normal.normalized();

                const stl_triangle_vertex_indices& indices = its.indices[face];
                const Vec3d a        = its.vertices[indices[0]].cast<double>();
                const Vec3d b        = its.vertices[indices[1]].cast<double>();
                const Vec3d c        = its.vertices[indices[2]].cast<double>();
                const Vec3d centroid = (a + b + c) / 3.;
                const std::array<Vec3d, 4> points{
                    centroid,
                    0.5 * (centroid + a),
                    0.5 * (centroid + b),
                    0.5 * (centroid + c)
                };
                // The rays after the first only need to look closer than the shortest so far.
                double shortest    = Infinity;
                bool inner         = false;
                const auto measure = [&](const Vec3d& point)
                {
                    const double distance =
                        measurement.thickness(int(face), point, direction, shortest, scratch);
                    if (distance == Inner)
                        inner = true;
                    else
                        shortest = std::min(shortest, distance);
                };
                for (const Vec3d& point : points)
                    measure(point);
                // A triangle partly inside of another shell, as of a pin through a block, may stick
                // out near its corners.
                if (inner)
                    for (const Vec3d& corner : {a, b, c})
                        measure(0.1 * centroid + 0.9 * corner);
                thickness[face] = float(shortest);
            }
        }
    );
    return thickness;
}

} // namespace Slic3r::Biz::Algorithms::MeshFix

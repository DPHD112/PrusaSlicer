#include "Slic3r/Biz/Algorithms/MeshFix/Session.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "Slic3r/Biz/Algorithms/MeshFix/Edit.hpp"
#include "Slic3r/Biz/Algorithms/TriangleMesh.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix {

using Domain::Transform3d;
using Domain::Vec3d;
using Domain::Vec3f;

namespace {

size_t count_marked(const FaceMask& mask)
{
    return size_t(std::count(mask.begin(), mask.end(), true));
}

// Closest point of a triangle to a point (C. Ericson: Real-Time Collision Detection, 5.1.5).
Vec3d closest_point_on_triangle(const Vec3d& p, const Vec3d& a, const Vec3d& b, const Vec3d& c)
{
    const Vec3d ab  = b - a;
    const Vec3d ac  = c - a;
    const Vec3d ap  = p - a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0. && d2 <= 0.)
        return a;
    const Vec3d bp  = p - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0. && d4 <= d3)
        return b;
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0. && d1 >= 0. && d3 <= 0.)
        return a + ab * (d1 / (d1 - d3));
    const Vec3d cp  = p - c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0. && d5 <= d6)
        return c;
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0. && d2 >= 0. && d6 <= 0.)
        return a + ac * (d2 / (d2 - d6));
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0. && (d4 - d3) >= 0. && (d5 - d6) >= 0.)
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const double denominator = 1. / (va + vb + vc);
    return a + ab * (vb * denominator) + ac * (vc * denominator);
}

// The sphere is given in the coordinates the mesh is transformed to.
bool touches_sphere(
    const indexed_triangle_set& its,
    int face,
    const Transform3d& trafo,
    const Vec3d& center,
    double radius
)
{
    const stl_triangle_vertex_indices& indices = its.indices[face];
    const Vec3d closest                        = closest_point_on_triangle(
        center,
        trafo * its.vertices[indices[0]].cast<double>(),
        trafo * its.vertices[indices[1]].cast<double>(),
        trafo * its.vertices[indices[2]].cast<double>()
    );
    return (closest - center).squaredNorm() <= radius * radius;
}

// Triangles touching the sphere and connected to the seed through triangles touching the sphere,
// so that the brush does not reach through thin walls.
FaceMask brush_region(
    const indexed_triangle_set& its,
    const MeshTopology& topology,
    int seed_face,
    const Transform3d& trafo,
    const Vec3d& center,
    double radius
)
{
    FaceMask mask(topology.faces_count(), false);
    std::vector<int> queue{seed_face};
    mask[seed_face] = true;
    while (!queue.empty()) {
        const int face = queue.back();
        queue.pop_back();
        for (int edge = 0; edge < 3; ++edge) {
            const int edge_idx = topology.edge_index(face, edge);
            if (edge_idx < 0)
                continue;
            for (const FaceEdge& side : topology.edge_faces(edge_idx))
                if (!mask[side.face] && touches_sphere(its, side.face, trafo, center, radius)) {
                    mask[side.face] = true;
                    queue.push_back(side.face);
                }
        }
    }
    return mask;
}

// All vertices lie in a plane, such a piece has no volume.
bool is_flat(const indexed_triangle_set& its)
{
    if (its.vertices.size() < 4)
        return true;
    // The vertex farthest from the first one and the vertex farthest from the line through both
    // span the plane.
    const Vec3d a = its.vertices.front().cast<double>();
    Vec3d b       = a;
    for (const Vec3f& vertex : its.vertices)
        if ((vertex.cast<double>() - a).squaredNorm() > (b - a).squaredNorm())
            b = vertex.cast<double>();
    const double size = (b - a).norm();
    if (size == 0.)
        return true;
    const Vec3d axis  = (b - a) / size;
    Vec3d c           = a;
    double c_distance = 0.;
    for (const Vec3f& vertex : its.vertices) {
        const Vec3d offset    = vertex.cast<double>() - a;
        const double distance = (offset - axis * axis.dot(offset)).squaredNorm();
        if (distance > c_distance) {
            c          = vertex.cast<double>();
            c_distance = distance;
        }
    }
    // Float coordinates far from the origin are not exactly coplanar.
    const double tolerance = 1e-5 * (size + a.cwiseAbs().maxCoeff());
    if (std::sqrt(c_distance) <= tolerance)
        return true;
    const Vec3d normal = axis.cross(c - a).normalized();
    return std::all_of(
        its.vertices.begin(),
        its.vertices.end(),
        [&](const Vec3f& vertex)
        { return std::abs(normal.dot(vertex.cast<double>() - a)) <= tolerance; }
    );
}

// Number of shells containing a marked triangle.
size_t count_marked_shells(const MeshTopology& topology, const FaceMask& mask)
{
    const Shells shells = find_shells(topology);
    std::vector<bool> marked_shell(shells.count(), false);
    for (size_t face = 0; face < mask.size(); ++face)
        if (mask[face])
            marked_shell[shells.face_shell[face]] = true;
    return size_t(std::count(marked_shell.begin(), marked_shell.end(), true));
}

} // namespace

Session::Session(indexed_triangle_set mesh)
{
    reset(std::move(mesh));
}

void Session::reset(indexed_triangle_set mesh)
{
    m_mesh = std::move(mesh);
    update_after_edit(false);
}

void Session::update_after_edit(bool keep_marks)
{
    // Unreferenced vertices would break the triangle indexing of the render geometry.
    TriangleMesh::its_compactify_vertices(m_mesh);
    m_topology = std::make_unique<MeshTopology>(m_mesh);

    const size_t faces_count = m_mesh.indices.size();
    if (keep_marks)
        m_marked.resize(faces_count, false);
    else
        m_marked.assign(faces_count, false);
    update_marked_count();

    m_holes   = find_holes(m_mesh, *m_topology);
    m_flipped = MeshFix::mark_flipped_faces(m_mesh, *m_topology);
    m_noise   = MeshFix::mark_noise_shells(m_mesh, *m_topology);
    m_intersecting.assign(faces_count, false);
    m_intersections_detected = false;

    m_diagnostics                    = MeshDiagnostics{};
    m_diagnostics.faces              = faces_count;
    m_diagnostics.vertices           = m_mesh.vertices.size();
    m_diagnostics.degenerate_faces   = count_marked(mark_degenerate_faces(m_mesh));
    m_diagnostics.open_edges         = m_topology->count(EdgeKind::Open);
    m_diagnostics.non_manifold_edges = m_topology->count(EdgeKind::NonManifold);
    m_diagnostics.inconsistent_edges = m_topology->count(EdgeKind::Inconsistent);
    m_diagnostics.flipped_faces      = count_marked(m_flipped);
    m_diagnostics.holes              = m_holes.size();
    m_diagnostics.shells             = find_shells(*m_topology).count();
    m_diagnostics.noise_shells       = count_marked_shells(*m_topology, m_noise);
}

void Session::update_marked_count()
{
    m_marked_count = count_marked(m_marked);
}

FaceMask
Session::region(MarkTool tool, int face, const Vec3f& point, const MarkParams& params) const
{
    if (face < 0 || size_t(face) >= m_mesh.indices.size())
        return FaceMask(m_mesh.indices.size(), false);

    switch (tool) {
    case MarkTool::Triangle: {
        FaceMask mask(m_mesh.indices.size(), false);
        mask[face] = true;
        return mask;
    }
    case MarkTool::Shell:
        return mark_shell(*m_topology, face);
    case MarkTool::Surface:
        return mark_surface(m_mesh, *m_topology, face, params.surface_angle_deg);
    case MarkTool::Plane:
        return mark_plane(m_mesh, *m_topology, face, params.plane_angle_deg, params.plane_distance);
    case MarkTool::Brush:
        return brush_region(
            m_mesh,
            *m_topology,
            face,
            params.brush_trafo,
            params.brush_trafo * point.cast<double>(),
            double(params.brush_radius)
        );
    }
    return FaceMask(m_mesh.indices.size(), false);
}

bool Session::mark(const FaceMask& region, bool add)
{
    bool changed = false;
    for (size_t face = 0; face < m_marked.size() && face < region.size(); ++face)
        if (region[face] && m_marked[face] != add) {
            m_marked[face] = add;
            changed        = true;
        }
    if (changed)
        update_marked_count();
    return changed;
}

void Session::mark_all()
{
    m_marked.assign(m_mesh.indices.size(), true);
    update_marked_count();
}

void Session::clear_marks()
{
    m_marked.assign(m_mesh.indices.size(), false);
    update_marked_count();
}

void Session::invert_marks()
{
    m_marked.flip();
    update_marked_count();
}

void Session::grow_marks()
{
    m_marked = grow_marking(m_mesh, m_marked);
    update_marked_count();
}

void Session::shrink_marks()
{
    m_marked = shrink_marking(m_mesh, m_marked);
    update_marked_count();
}

void Session::mark_bad_edge_faces()
{
    mark(MeshFix::mark_bad_edge_faces(*m_topology), true);
}

void Session::mark_flipped_faces()
{
    mark(m_flipped, true);
}

void Session::mark_noise_shells()
{
    mark(m_noise, true);
}

void Session::mark_intersecting_faces()
{
    if (!m_intersections_detected)
        detect_intersections();
    mark(m_intersecting, true);
}

int Session::hole_at_ray(
    const Vec3d& origin,
    const Vec3d& direction,
    int hit_face,
    double hit_distance
) const
{
    const double direction_length = direction.norm();
    if (direction_length <= 0.)
        return -1;
    const Vec3d unit_direction = direction / direction_length;

    // A hole the ray passes through in front of the hit surface. The hole is approximated
    // by the sphere around the centroid of its boundary.
    int best_hole      = -1;
    double best_offset = std::numeric_limits<double>::max();
    for (size_t hole_idx = 0; hole_idx < m_holes.size(); ++hole_idx) {
        const std::vector<int>& vertices = m_holes[hole_idx].vertices;
        Vec3d centroid                   = Vec3d::Zero();
        for (int vertex_idx : vertices)
            centroid += m_mesh.vertices[vertex_idx].cast<double>();
        centroid /= double(vertices.size());
        double radius_squared = 0.;
        for (int vertex_idx : vertices)
            radius_squared = std::max(
                radius_squared,
                (m_mesh.vertices[vertex_idx].cast<double>() - centroid).squaredNorm()
            );

        const double offset = (centroid - origin).dot(unit_direction);
        if (offset < 0. || offset >= best_offset)
            continue;
        if (hit_face >= 0 && offset > hit_distance + 0.25 * std::sqrt(radius_squared))
            continue;
        if ((origin + offset * unit_direction - centroid).squaredNorm() <= radius_squared) {
            best_hole   = int(hole_idx);
            best_offset = offset;
        }
    }
    if (best_hole >= 0 || hit_face < 0 || size_t(hit_face) >= m_mesh.indices.size())
        return best_hole;

    // The hole bordering the hit triangle.
    for (int edge = 0; edge < 3; ++edge) {
        const int edge_idx = m_topology->edge_index(hit_face, edge);
        if (edge_idx >= 0 && m_topology->edge_kind(edge_idx) == EdgeKind::Open) {
            const Domain::Index2& vertices = m_topology->edge_vertices(edge_idx);
            const int hole_idx             = find_hole_by_edge(m_holes, vertices[0], vertices[1]);
            if (hole_idx >= 0)
                return hole_idx;
        }
    }
    return -1;
}

size_t Session::delete_marked()
{
    if (m_marked_count == 0 || m_marked_count == m_mesh.indices.size())
        return 0;
    const size_t removed = delete_faces(m_mesh, m_marked);
    update_after_edit(false);
    return removed;
}

size_t Session::flip_marked()
{
    const size_t flipped = flip_faces(m_mesh, m_marked);
    if (flipped > 0)
        update_after_edit(true);
    return flipped;
}

indexed_triangle_set Session::split_marked()
{
    if (m_marked_count == 0 || m_marked_count == m_mesh.indices.size())
        return {};
    indexed_triangle_set remaining = m_mesh;
    indexed_triangle_set extracted = extract_faces(remaining, m_marked);
    if (is_flat(extracted))
        return {};
    m_mesh = std::move(remaining);
    update_after_edit(false);
    return extracted;
}

size_t Session::fill_hole(int hole_index, HoleFillMode mode)
{
    if (hole_index < 0 || size_t(hole_index) >= m_holes.size())
        return 0;
    const size_t added = MeshFix::fill_hole(m_mesh, m_holes[hole_index], {mode}).size();
    if (added > 0)
        update_after_edit(true);
    return added;
}

size_t Session::fill_all_holes(HoleFillMode mode)
{
    const size_t added = fill_holes(m_mesh, m_holes, {mode}).new_faces.size();
    if (added > 0)
        update_after_edit(true);
    return added;
}

size_t Session::unify_normals()
{
    const size_t flipped = flip_faces(m_mesh, m_flipped);
    if (flipped > 0)
        update_after_edit(true);
    return flipped;
}

size_t Session::stitch(float tolerance)
{
    indexed_triangle_set stitched = m_mesh;
    const size_t merged           = stitch_open_edges(stitched, tolerance);
    // Never remove the whole mesh.
    if (merged == 0 || stitched.indices.empty())
        return 0;
    m_mesh = std::move(stitched);
    update_after_edit(false);
    return merged;
}

size_t Session::remove_noise_shells(const NoiseShellParams& params)
{
    const FaceMask noise = MeshFix::mark_noise_shells(m_mesh, *m_topology, params);
    // Never remove the whole mesh.
    if (count_marked(noise) == m_mesh.indices.size())
        return 0;
    const size_t removed = delete_faces(m_mesh, noise);
    if (removed > 0)
        update_after_edit(false);
    return removed;
}

AutoFixReport Session::auto_fix(const AutoFixParams& params)
{
    indexed_triangle_set fixed = m_mesh;
    const AutoFixReport report = MeshFix::auto_fix(fixed, params);
    // Never remove the whole mesh.
    if (!report.changed() || fixed.indices.empty())
        return {};
    m_mesh = std::move(fixed);
    update_after_edit(false);
    return report;
}

void Session::detect_intersections()
{
    m_intersecting                   = MeshFix::mark_intersecting_faces(m_mesh);
    m_intersections_detected         = true;
    m_diagnostics.intersecting_faces = count_marked(m_intersecting);
}

} // namespace Slic3r::Biz::Algorithms::MeshFix

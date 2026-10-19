#include "Slic3r/Biz/Algorithms/MeshFix/HoleFill.hpp"

#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numbers>
#include <tuple>
#include <unordered_map>

#include "Slic3r/Domain/Types.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix {

using Domain::Index3;
using Domain::Vec3d;

namespace {

// Neighbor vertices and incident triangles of the hole boundary vertices.
class BoundaryRings
{
public:
    BoundaryRings(const indexed_triangle_set& its, const std::vector<Hole>& holes) :
        m_slot(its.vertices.size(), -1)
    {
        for (const Hole& hole : holes)
            for (int vertex_idx : hole.vertices)
                if (vertex_idx >= 0 && size_t(vertex_idx) < m_slot.size() && m_slot[vertex_idx] < 0)
                {
                    m_slot[vertex_idx] = int(m_rings.size());
                    m_rings.emplace_back();
                    m_faces.emplace_back();
                }
        add_faces(its, 0);
    }

    // Register the triangles its.indices[first_face, end) incident with the boundary vertices.
    void add_faces(const indexed_triangle_set& its, size_t first_face)
    {
        for (size_t face = first_face; face < its.indices.size(); ++face) {
            const stl_triangle_vertex_indices& indices = its.indices[face];
            for (int i = 0; i < 3; ++i) {
                const int vertex_idx = indices[i];
                if (!is_tracked(vertex_idx))
                    continue;
                m_faces[m_slot[vertex_idx]].push_back(int(face));
                std::vector<int>& ring = m_rings[m_slot[vertex_idx]];
                for (int other : {indices[(i + 1) % 3], indices[(i + 2) % 3]})
                    if (other != vertex_idx
                        && std::find(ring.begin(), ring.end(), other) == ring.end())
                        ring.push_back(other);
            }
        }
    }

    bool is_tracked(int vertex_idx) const
    {
        return vertex_idx >= 0 && size_t(vertex_idx) < m_slot.size() && m_slot[vertex_idx] >= 0;
    }

    const std::vector<int>& ring(int vertex_idx) const
    {
        return m_rings[m_slot[vertex_idx]];
    }

    const std::vector<int>& faces(int vertex_idx) const
    {
        return m_faces[m_slot[vertex_idx]];
    }

    bool has_edge(int boundary_vertex, int other) const
    {
        const std::vector<int>& vertex_ring = ring(boundary_vertex);
        return std::find(vertex_ring.begin(), vertex_ring.end(), other) != vertex_ring.end();
    }

private:
    std::vector<int> m_slot;
    std::vector<std::vector<int>> m_rings;
    std::vector<std::vector<int>> m_faces;
};

struct HoleContext
{
    const Hole& hole;
    const BoundaryRings& rings;
    // Unit normals of the existing triangles along the hole sides, zero when not defined.
    std::vector<Vec3d> side_normals;

    // Is there already a mesh edge between the i-th and j-th boundary vertex?
    bool is_mesh_edge(int i, int j) const
    {
        return rings.has_edge(hole.vertices[i], hole.vertices[j]);
    }
};

struct Patch
{
    // Hole boundary vertices first, then the added vertices.
    std::vector<Vec3d> points;
    std::vector<Index3> faces;
    size_t boundary_size{0};
};

Vec3d unit_normal(const Vec3d& a, const Vec3d& b, const Vec3d& c)
{
    const Vec3d normal  = (b - a).cross(c - a);
    const double length = normal.norm();
    return length > 0. ? Vec3d(normal / length) : Vec3d::Zero();
}

Vec3d unit_normal(const std::vector<Vec3d>& points, const Index3& face)
{
    return unit_normal(points[face[0]], points[face[1]], points[face[2]]);
}

// 0 for coplanar neighbor triangles, 2 for neighbors folded onto each other.
double fold_measure(const Vec3d& normal, const Vec3d& neighbor_normal)
{
    if (normal.isZero())
        return 2.;
    if (neighbor_normal.isZero())
        return 0.;
    return 1. - normal.dot(neighbor_normal);
}

double angle_at(const Vec3d& apex, const Vec3d& a, const Vec3d& b)
{
    const Vec3d u = a - apex;
    const Vec3d v = b - apex;
    return std::atan2(u.cross(v).norm(), u.dot(v));
}

struct TriangulationWeight
{
    // The largest fold_measure() between neighbor triangles.
    double fold{0.};
    double area{0.};
};

bool is_better(const TriangulationWeight& lhs, const TriangulationWeight& rhs)
{
    constexpr double fold_epsilon = 1e-9;
    if (lhs.fold < rhs.fold - fold_epsilon)
        return true;
    if (lhs.fold > rhs.fold + fold_epsilon)
        return false;
    return lhs.area < rhs.area;
}

/**
 * Triangulation of a polygon in 3D minimizing the largest dihedral angle and then the area,
 * see P. Liepa: Filling Holes in Meshes, 2003. Takes O(n^3) time.
 */
bool triangulate_optimal(
    const HoleContext& context,
    const std::vector<Vec3d>& points,
    std::vector<Index3>& faces
)
{
    const int n           = int(points.size());
    const double infinite = std::numeric_limits<double>::infinity();
    auto at               = [n](int i, int j) { return size_t(i) * size_t(n) + size_t(j); };

    // Best triangulation of the polygon points[i], ..., points[j] closed by the side j -> i.
    std::vector<TriangulationWeight> weights(size_t(n) * size_t(n));
    // The third vertex of the triangle on the side i -> j.
    std::vector<int> splits(size_t(n) * size_t(n), -1);
    // Normal of the triangle on the side i -> j.
    std::vector<Vec3d> normals(size_t(n) * size_t(n), Vec3d::Zero());
    // Normal of the triangle across the side i -> m from the triangle (i, m, j).
    auto neighbor_normal = [&](int i, int m) -> const Vec3d&
    { return m == i + 1 ? context.side_normals[i] : normals[at(i, m)]; };

    for (int length = 2; length < n; ++length) {
        for (int i = 0; i + length < n; ++i) {
            const int j               = i + length;
            TriangulationWeight& best = weights[at(i, j)];
            best                      = {infinite, infinite};
            const bool is_root        = i == 0 && j == n - 1;
            // A new edge would duplicate an existing one, creating a non-manifold edge.
            if (!is_root && context.is_mesh_edge(i, j))
                continue;
            for (int m = i + 1; m < j; ++m) {
                const TriangulationWeight& left  = weights[at(i, m)];
                const TriangulationWeight& right = weights[at(m, j)];
                if (left.fold == infinite || right.fold == infinite)
                    continue;
                const Vec3d normal = unit_normal(points[i], points[m], points[j]);
                double fold        = std::max(
                    {left.fold,
                            right.fold,
                            fold_measure(normal, neighbor_normal(i, m)),
                            fold_measure(normal, neighbor_normal(m, j))}
                );
                if (is_root)
                    fold = std::max(fold, fold_measure(normal, context.side_normals[n - 1]));
                const double area =
                    0.5 * (points[m] - points[i]).cross(points[j] - points[i]).norm();
                const TriangulationWeight weight{fold, left.area + right.area + area};
                if (is_better(weight, best)) {
                    best              = weight;
                    splits[at(i, j)]  = m;
                    normals[at(i, j)] = normal;
                }
            }
        }
    }

    if (splits[at(0, n - 1)] < 0)
        return false;

    std::vector<std::pair<int, int>> stack{{0, n - 1}};
    while (!stack.empty()) {
        const auto [i, j] = stack.back();
        stack.pop_back();
        const int m = splits[at(i, j)];
        faces.push_back({i, m, j});
        if (m - i > 1)
            stack.emplace_back(i, m);
        if (j - m > 1)
            stack.emplace_back(m, j);
    }
    return true;
}

// Ear clipping, the sharpest convex corner first. Takes O(n^2) time.
void triangulate_greedy(
    const HoleContext& context,
    const std::vector<Vec3d>& points,
    std::vector<Index3>& faces
)
{
    const size_t n = points.size();
    // Newell's normal, oriented as the patch triangles.
    Vec3d loop_normal = Vec3d::Zero();
    for (size_t i = 0; i < n; ++i)
        loop_normal += points[i].cross(points[(i + 1) % n]);

    std::vector<int> polygon(n);
    for (size_t i = 0; i < n; ++i)
        polygon[i] = int(i);

    while (polygon.size() > 3) {
        const size_t size = polygon.size();
        size_t best       = 0;
        double best_score = std::numeric_limits<double>::infinity();
        for (size_t k = 0; k < size; ++k) {
            const int prev     = polygon[(k + size - 1) % size];
            const int current  = polygon[k];
            const int next     = polygon[(k + 1) % size];
            const double angle = angle_at(points[current], points[prev], points[next]);
            const bool convex =
                (points[current] - points[prev]).cross(points[next] - points[prev]).dot(loop_normal)
                >= 0.;
            double score = convex ? angle : 2. * std::numbers::pi - angle;
            // Avoid duplicating an existing mesh edge, unless there is no other choice.
            if (context.is_mesh_edge(prev, next))
                score += 4. * std::numbers::pi;
            if (score < best_score) {
                best_score = score;
                best       = k;
            }
        }
        faces.push_back(
            {polygon[(best + size - 1) % size], polygon[best], polygon[(best + 1) % size]}
        );
        polygon.erase(polygon.begin() + best);
    }
    faces.push_back({polygon[0], polygon[1], polygon[2]});
}

uint64_t edge_key(int vertex_a, int vertex_b)
{
    const auto [v0, v1] = std::minmax(vertex_a, vertex_b);
    return (uint64_t(uint32_t(v0)) << 32) | uint64_t(uint32_t(v1));
}

struct PatchEdge
{
    uint64_t key;
    std::array<int, 2> faces;
    std::array<int, 2> sides;
    int count;
};

// Edges of the patch sorted by key.
std::vector<PatchEdge> collect_edges(const std::vector<Index3>& faces)
{
    std::vector<std::tuple<uint64_t, int, int>> sides;
    sides.reserve(3 * faces.size());
    for (size_t face = 0; face < faces.size(); ++face)
        for (int side = 0; side < 3; ++side)
            sides.emplace_back(
                edge_key(faces[face][side], faces[face][(side + 1) % 3]),
                int(face),
                side
            );
    std::sort(sides.begin(), sides.end());

    std::vector<PatchEdge> edges;
    for (const auto& [key, face, side] : sides) {
        if (edges.empty() || edges.back().key != key)
            edges.push_back({key, {-1, -1}, {-1, -1}, 0});
        PatchEdge& edge = edges.back();
        if (edge.count < 2) {
            edge.faces[edge.count] = face;
            edge.sides[edge.count] = side;
        }
        ++edge.count;
    }
    return edges;
}

bool contains_edge(const std::vector<PatchEdge>& edges, uint64_t key)
{
    auto it = std::lower_bound(
        edges.begin(),
        edges.end(),
        key,
        [](const PatchEdge& edge, uint64_t k) { return edge.key < k; }
    );
    return it != edges.end() && it->key == key;
}

Index3 rotated(const Index3& face, int first)
{
    return {face[first], face[(first + 1) % 3], face[(first + 2) % 3]};
}

/**
 * Flip edges to improve the triangle shapes (Delaunay criterion).
 * When planar_only is set, only edges between nearly coplanar triangles are flipped,
 * so that the shape of the patch is not changed.
 */
void flip_edges(Patch& patch, const HoleContext& context, bool planar_only)
{
    constexpr double min_planar_cos  = 0.9999;
    const int boundary_size          = int(patch.boundary_size);
    const std::vector<Vec3d>& points = patch.points;
    for (int pass = 0; pass < 64; ++pass) {
        const std::vector<PatchEdge> edges = collect_edges(patch.faces);
        std::vector<uint64_t> created_edges;
        std::vector<bool> touched(patch.faces.size(), false);
        bool flipped = false;
        for (const PatchEdge& edge : edges) {
            if (edge.count != 2 || touched[edge.faces[0]] || touched[edge.faces[1]])
                continue;
            const Index3 first  = rotated(patch.faces[edge.faces[0]], edge.sides[0]);
            const Index3 second = rotated(patch.faces[edge.faces[1]], edge.sides[1]);
            const int a         = first[0];
            const int b         = first[1];
            const int c         = first[2];
            const int d         = second[2];
            if (second[0] != b || second[1] != a || c == d)
                continue;
            if (angle_at(points[c], points[a], points[b])
                    + angle_at(points[d], points[b], points[a])
                <= std::numbers::pi + 1e-9)
                continue;
            const uint64_t new_key = edge_key(c, d);
            if (contains_edge(edges, new_key)
                || std::find(created_edges.begin(), created_edges.end(), new_key)
                    != created_edges.end()
                || (c < boundary_size && d < boundary_size && context.is_mesh_edge(c, d)))
                continue;

            const Index3 new_first{c, a, d};
            const Index3 new_second{d, b, c};
            const Vec3d first_normal      = unit_normal(points, first);
            const Vec3d second_normal     = unit_normal(points, second);
            const Vec3d new_first_normal  = unit_normal(points, new_first);
            const Vec3d new_second_normal = unit_normal(points, new_second);
            if (planar_only) {
                if (first_normal.dot(second_normal) < min_planar_cos
                    || new_first_normal.dot(new_second_normal) < min_planar_cos)
                    continue;
            }
            // Do not fold the patch.
            const Vec3d old_normal = first_normal + second_normal;
            if (new_first_normal.dot(old_normal) <= 0. || new_second_normal.dot(old_normal) <= 0.)
                continue;

            patch.faces[edge.faces[0]] = new_first;
            patch.faces[edge.faces[1]] = new_second;
            touched[edge.faces[0]]     = true;
            touched[edge.faces[1]]     = true;
            created_edges.push_back(new_key);
            flipped = true;
        }
        if (!flipped)
            break;
    }
}

/**
 * Insert vertices at triangle centroids until the triangles are of size given by the vertex scales,
 * relaxing the edges after each round, see P. Liepa: Filling Holes in Meshes, 2003.
 */
void refine(Patch& patch, const HoleContext& context, std::vector<double> scales)
{
    constexpr size_t max_faces = 200000;
    const double density       = std::sqrt(2.);
    for (int round = 0; round < 100; ++round) {
        bool split               = false;
        const size_t faces_count = patch.faces.size();
        for (size_t face_idx = 0; face_idx < faces_count && patch.faces.size() < max_faces;
             ++face_idx)
        {
            const Index3 face = patch.faces[face_idx];
            const Vec3d centroid =
                (patch.points[face[0]] + patch.points[face[1]] + patch.points[face[2]]) / 3.;
            const double centroid_scale =
                (scales[face[0]] + scales[face[1]] + scales[face[2]]) / 3.;
            bool split_face = true;
            for (int vertex_idx : face) {
                const double distance = density * (centroid - patch.points[vertex_idx]).norm();
                if (distance <= centroid_scale || distance <= scales[vertex_idx]) {
                    split_face = false;
                    break;
                }
            }
            if (!split_face)
                continue;
            const int center = int(patch.points.size());
            patch.points.push_back(centroid);
            scales.push_back(centroid_scale);
            patch.faces[face_idx] = {face[0], face[1], center};
            patch.faces.push_back({face[1], face[2], center});
            patch.faces.push_back({face[2], face[0], center});
            split = true;
        }
        if (!split)
            break;
        flip_edges(patch, context, false);
        if (patch.faces.size() >= max_faces)
            break;
    }
}

// Neighbors of a vertex with the weights of the connecting edges.
using WeightedNeighbors = std::vector<std::pair<int, double>>;

/**
 * Cotangent weights of the edges of a triangle set, clamped to stay positive.
 * Unlike the uniform weights, they do not distort the fairing where the triangles
 * around a vertex are distributed unevenly, typically at the hole boundary.
 */
std::vector<WeightedNeighbors>
cotangent_weights(const std::vector<Vec3d>& positions, const std::vector<Index3>& triangles)
{
    constexpr double min_weight = 0.01;
    std::vector<WeightedNeighbors> weights(positions.size());
    auto add = [&weights](int from, int to, double weight)
    {
        WeightedNeighbors& neighbors = weights[from];
        auto it                      = std::find_if(
            neighbors.begin(),
            neighbors.end(),
            [to](const auto& item) { return item.first == to; }
        );
        if (it == neighbors.end())
            neighbors.emplace_back(to, weight);
        else
            it->second += weight;
    };
    for (const Index3& triangle : triangles)
        for (int side = 0; side < 3; ++side) {
            const int a             = triangle[side];
            const int b             = triangle[(side + 1) % 3];
            const Vec3d u           = positions[a] - positions[triangle[(side + 2) % 3]];
            const Vec3d v           = positions[b] - positions[triangle[(side + 2) % 3]];
            const double cross_norm = u.cross(v).norm();
            const double cotangent  = cross_norm > 0. ? u.dot(v) / cross_norm : 0.;
            add(a, b, 0.5 * cotangent);
            add(b, a, 0.5 * cotangent);
        }
    for (WeightedNeighbors& neighbors : weights)
        for (auto& [neighbor, weight] : neighbors)
            weight = std::max(weight, min_weight);
    return weights;
}

/**
 * Move the added vertices to minimize the discrete Laplacian (membrane, smooth = false)
 * or its square (thin plate, smooth = true). The thin plate fairing includes the mesh triangles
 * around the hole to continue the curvature of the surrounding surface, it is repeated with
 * the weights updated to the new shape of the patch.
 */
void fair(Patch& patch, const HoleContext& context, const indexed_triangle_set& its, bool smooth)
{
    const int boundary_size = int(patch.boundary_size);
    const int patch_size    = int(patch.points.size());
    const int unknowns      = patch_size - boundary_size;
    if (unknowns <= 0)
        return;

    // Patch vertices first, then the fixed mesh vertices around the hole.
    std::vector<Vec3d> positions(patch.points);
    std::vector<Index3> triangles(patch.faces);
    if (smooth) {
        std::unordered_map<int, int> local_index;
        for (int i = 0; i < boundary_size; ++i)
            local_index.emplace(context.hole.vertices[i], i);
        std::vector<int> mesh_faces;
        for (int vertex_idx : context.hole.vertices)
            mesh_faces.insert(
                mesh_faces.end(),
                context.rings.faces(vertex_idx).begin(),
                context.rings.faces(vertex_idx).end()
            );
        std::sort(mesh_faces.begin(), mesh_faces.end());
        mesh_faces.erase(std::unique(mesh_faces.begin(), mesh_faces.end()), mesh_faces.end());
        for (int face : mesh_faces) {
            Index3 triangle;
            for (int i = 0; i < 3; ++i) {
                const int mesh_vertex = its.indices[face][i];
                auto [it, inserted]   = local_index.try_emplace(mesh_vertex, int(positions.size()));
                if (inserted)
                    positions.push_back(its.vertices[mesh_vertex].cast<double>());
                triangle[i] = it->second;
            }
            triangles.push_back(triangle);
        }
    }

    const int iterations = smooth ? 3 : 1;
    for (int iteration = 0; iteration < iterations; ++iteration) {
        const std::vector<WeightedNeighbors> weights = cotangent_weights(positions, triangles);
        auto total_weight                            = [&weights](int vertex)
        {
            double total = 0.;
            for (const auto& [neighbor, weight] : weights[vertex])
                total += weight;
            return total;
        };

        std::vector<Eigen::Triplet<double>> triplets;
        Eigen::MatrixX3d rhs = Eigen::MatrixX3d::Zero(unknowns, 3);
        std::map<int, double> row;
        // Barycentric vertex areas.
        std::vector<double> areas(positions.size(), 0.);
        for (const Index3& triangle : triangles) {
            const double area =
                (positions[triangle[1]] - positions[triangle[0]])
                    .cross(positions[triangle[2]] - positions[triangle[0]])
                    .norm()
                / 6.;
            for (int vertex : triangle)
                areas[vertex] += area;
        }
        // Adds scale * L(vertex) to the row, L(vertex) = sum(w_i * (x_i - x)) / area.
        auto add_laplacian = [&](int vertex, double scale)
        {
            const double area = std::max(areas[vertex], 1e-12);
            for (const auto& [neighbor, weight] : weights[vertex])
                row[neighbor] += scale * weight / area;
            row[vertex] -= scale * total_weight(vertex) / area;
        };
        for (int r = 0; r < unknowns; ++r) {
            const int vertex = boundary_size + r;
            row.clear();
            if (smooth) {
                // L(L(vertex)) multiplied by the area.
                for (const auto& [neighbor, weight] : weights[vertex])
                    add_laplacian(neighbor, weight);
                add_laplacian(vertex, -total_weight(vertex));
            } else {
                // L(vertex) multiplied by the area, which makes the system symmetric.
                add_laplacian(vertex, std::max(areas[vertex], 1e-12));
            }
            for (const auto& [column_vertex, coefficient] : row) {
                if (column_vertex >= boundary_size && column_vertex < patch_size)
                    triplets.emplace_back(r, column_vertex - boundary_size, coefficient);
                else
                    rhs.row(r) -= coefficient * positions[column_vertex].transpose();
            }
        }

        Eigen::SparseMatrix<double> matrix(unknowns, unknowns);
        matrix.setFromTriplets(triplets.begin(), triplets.end());
        Eigen::MatrixX3d solution;
        if (smooth) {
            Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
            solver.compute(matrix);
            if (solver.info() != Eigen::Success)
                break;
            solution = solver.solve(rhs);
            if (solver.info() != Eigen::Success)
                break;
        } else {
            // The negative Laplacian restricted to the interior vertices is symmetric positive definite.
            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
            solver.compute(-matrix);
            if (solver.info() != Eigen::Success)
                break;
            solution = solver.solve(-rhs);
            if (solver.info() != Eigen::Success)
                break;
        }
        if (!solution.allFinite())
            break;
        for (int r = 0; r < unknowns; ++r)
            positions[boundary_size + r] = solution.row(r).transpose();
    }

    std::copy(
        positions.begin() + boundary_size,
        positions.begin() + patch_size,
        patch.points.begin() + boundary_size
    );
}

std::vector<double> boundary_scales(const indexed_triangle_set& its, const HoleContext& context)
{
    const std::vector<int>& vertices = context.hole.vertices;
    const size_t n                   = vertices.size();
    std::vector<double> scales(n, 0.);
    for (size_t i = 0; i < n; ++i) {
        const Vec3d vertex           = its.vertices[vertices[i]].cast<double>();
        const std::vector<int>& ring = context.rings.ring(vertices[i]);
        double sum                   = 0.;
        for (int neighbor : ring)
            sum += (its.vertices[neighbor].cast<double>() - vertex).norm();
        scales[i] = ring.empty() ? 0. : sum / double(ring.size());
    }
    return scales;
}

std::vector<int> fill_single_hole(
    indexed_triangle_set& its,
    const Hole& hole,
    const BoundaryRings& rings,
    const HoleFillParams& params
)
{
    const size_t n = hole.size();
    if (n < 3 || hole.faces.size() != n)
        return {};
    for (size_t i = 0; i < n; ++i)
        if (!rings.is_tracked(hole.vertices[i])
            || hole.faces[i] < 0
            || size_t(hole.faces[i]) >= its.indices.size())
            return {};

    HoleContext context{hole, rings, {}};
    context.side_normals.reserve(n);
    for (int face : hole.faces) {
        const stl_triangle_vertex_indices& indices = its.indices[face];
        context.side_normals.push_back(unit_normal(
            its.vertices[indices[0]].cast<double>(),
            its.vertices[indices[1]].cast<double>(),
            its.vertices[indices[2]].cast<double>()
        ));
    }

    Patch patch;
    patch.boundary_size = n;
    patch.points.reserve(n);
    for (int vertex_idx : hole.vertices)
        patch.points.push_back(its.vertices[vertex_idx].cast<double>());

    if (n > params.max_optimal_triangulation_size
        || !triangulate_optimal(context, patch.points, patch.faces))
    {
        patch.faces.clear();
        triangulate_greedy(context, patch.points, patch.faces);
    }

    if (params.mode == HoleFillMode::Triangulate) {
        flip_edges(patch, context, true);
    } else {
        refine(patch, context, boundary_scales(its, context));
        fair(patch, context, its, params.mode == HoleFillMode::Smooth);
    }

    const int first_vertex = int(its.vertices.size());
    for (size_t i = n; i < patch.points.size(); ++i)
        its.vertices.push_back(patch.points[i].cast<float>());
    auto mesh_index = [&](int local)
    { return local < int(n) ? hole.vertices[local] : first_vertex + local - int(n); };

    std::vector<int> new_faces;
    new_faces.reserve(patch.faces.size());
    for (const Index3& face : patch.faces) {
        new_faces.push_back(int(its.indices.size()));
        its.indices.push_back({mesh_index(face[0]), mesh_index(face[1]), mesh_index(face[2])});
    }
    return new_faces;
}

// Split a closed walk along boundary sides into loops without repeated vertices.
void split_into_simple_loops(
    const std::vector<int>& walk_vertices,
    const std::vector<int>& walk_faces,
    std::vector<Hole>& holes
)
{
    Hole current;
    std::unordered_map<int, size_t> position;
    auto emit = [&holes](std::vector<int>&& vertices, std::vector<int>&& faces)
    {
        if (vertices.size() >= 3)
            holes.push_back({std::move(vertices), std::move(faces)});
    };
    for (size_t k = 0; k < walk_vertices.size(); ++k) {
        const int vertex = walk_vertices[k];
        auto it          = position.find(vertex);
        if (it != position.end()) {
            const size_t begin = it->second;
            for (size_t i = begin; i < current.vertices.size(); ++i)
                position.erase(current.vertices[i]);
            emit(
                std::vector<int>(current.vertices.begin() + begin, current.vertices.end()),
                std::vector<int>(current.faces.begin() + begin, current.faces.end())
            );
            current.vertices.resize(begin);
            current.faces.resize(begin);
        }
        position[vertex] = current.vertices.size();
        current.vertices.push_back(vertex);
        current.faces.push_back(walk_faces[k]);
    }
    emit(std::move(current.vertices), std::move(current.faces));
}

} // namespace

std::vector<Hole> find_holes(const indexed_triangle_set& its, const MeshTopology& topology)
{
    // A patch filling a hole traverses the open side a -> b of an existing triangle as b -> a.
    struct BoundarySide
    {
        int from;
        int to;
        int face;
    };

    std::vector<BoundarySide> sides;
    for (size_t edge_idx = 0; edge_idx < topology.edges_count(); ++edge_idx) {
        if (topology.edge_kind(int(edge_idx)) != EdgeKind::Open)
            continue;
        const FaceEdge& side                       = topology.edge_faces(int(edge_idx)).front();
        const stl_triangle_vertex_indices& indices = its.indices[side.face];
        sides.push_back({indices[(side.edge + 1) % 3], indices[side.edge], side.face});
    }
    std::sort(
        sides.begin(),
        sides.end(),
        [](const BoundarySide& lhs, const BoundarySide& rhs)
        { return std::tie(lhs.from, lhs.to, lhs.face) < std::tie(rhs.from, rhs.to, rhs.face); }
    );

    std::vector<bool> used(sides.size(), false);
    auto next_unused_side = [&](int vertex)
    {
        auto it = std::lower_bound(
            sides.begin(),
            sides.end(),
            vertex,
            [](const BoundarySide& side, int v) { return side.from < v; }
        );
        for (; it != sides.end() && it->from == vertex; ++it)
            if (!used[it - sides.begin()])
                return size_t(it - sides.begin());
        return sides.size();
    };

    std::vector<Hole> holes;
    std::vector<int> walk_vertices;
    std::vector<int> walk_faces;
    for (size_t first = 0; first < sides.size(); ++first) {
        if (used[first])
            continue;
        walk_vertices.clear();
        walk_faces.clear();
        const int start = sides[first].from;
        size_t current  = first;
        bool closed     = false;
        while (current < sides.size()) {
            used[current] = true;
            walk_vertices.push_back(sides[current].from);
            walk_faces.push_back(sides[current].face);
            if (sides[current].to == start) {
                closed = true;
                break;
            }
            current = next_unused_side(sides[current].to);
        }
        if (closed)
            split_into_simple_loops(walk_vertices, walk_faces, holes);
    }
    return holes;
}

double hole_perimeter(const indexed_triangle_set& its, const Hole& hole)
{
    double perimeter = 0.;
    for (size_t i = 0; i < hole.size(); ++i)
        perimeter +=
            (its.vertices[hole.vertices[(i + 1) % hole.size()]] - its.vertices[hole.vertices[i]])
                .cast<double>()
                .norm();
    return perimeter;
}

int find_hole_by_edge(const std::vector<Hole>& holes, int vertex_a, int vertex_b)
{
    for (size_t hole_idx = 0; hole_idx < holes.size(); ++hole_idx) {
        const std::vector<int>& vertices = holes[hole_idx].vertices;
        for (size_t i = 0; i < vertices.size(); ++i) {
            const int from = vertices[i];
            const int to   = vertices[(i + 1) % vertices.size()];
            if ((from == vertex_a && to == vertex_b) || (from == vertex_b && to == vertex_a))
                return int(hole_idx);
        }
    }
    return -1;
}

HoleFillResult
fill_holes(indexed_triangle_set& its, const std::vector<Hole>& holes, const HoleFillParams& params)
{
    HoleFillResult result;
    BoundaryRings rings(its, holes);
    for (const Hole& hole : holes) {
        const size_t first_face    = its.indices.size();
        std::vector<int> new_faces = fill_single_hole(its, hole, rings, params);
        if (new_faces.empty())
            continue;
        rings.add_faces(its, first_face);
        ++result.holes_filled;
        result.new_faces.insert(result.new_faces.end(), new_faces.begin(), new_faces.end());
    }
    return result;
}

std::vector<int>
fill_hole(indexed_triangle_set& its, const Hole& hole, const HoleFillParams& params)
{
    return fill_holes(its, {hole}, params).new_faces;
}

} // namespace Slic3r::Biz::Algorithms::MeshFix

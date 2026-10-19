#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <utility>
#include <vector>

#include "MeshFixTestUtils.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/Session.hpp"
#include "Slic3r/Biz/Algorithms/TriangleMesh.hpp"

using namespace Slic3r::Biz::Algorithms::MeshFix;
using namespace Slic3r::Biz::Algorithms::MeshFix::Test;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_cube;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_sphere;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_tetrahedron;
using Slic3r::Domain::Transform3d;
using Slic3r::Domain::Vec3d;
using Slic3r::Domain::Vec3f;

namespace {

// Sphere of radius 10 with the triangles above z = 7 removed.
indexed_triangle_set make_open_sphere()
{
    indexed_triangle_set sphere = its_make_sphere(10., 0.2);
    remove_faces_if(sphere, [&](size_t face) { return centroid(sphere, face).z() > 7.f; });
    return sphere;
}

// Square grid in the XZ plane centered at the origin.
indexed_triangle_set make_grid(float size, int cells)
{
    indexed_triangle_set grid;
    const float step = size / float(cells);
    for (int row = 0; row <= cells; ++row)
        for (int column = 0; column <= cells; ++column)
            grid.vertices.emplace_back(
                -0.5f * size + float(column) * step,
                0.f,
                -0.5f * size + float(row) * step
            );
    for (int row = 0; row < cells; ++row)
        for (int column = 0; column < cells; ++column) {
            const int corner = row * (cells + 1) + column;
            grid.indices.push_back({corner, corner + cells + 1, corner + 1});
            grid.indices.push_back({corner + 1, corner + cells + 1, corner + cells + 2});
        }
    return grid;
}

int face_at(const indexed_triangle_set& its, const Vec3f& point)
{
    int best            = -1;
    float best_distance = 0.f;
    for (size_t face = 0; face < its.indices.size(); ++face) {
        const float distance = (centroid(its, face) - point).norm();
        if (best < 0 || distance < best_distance) {
            best          = int(face);
            best_distance = distance;
        }
    }
    return best;
}

} // namespace

TEST_CASE("Session diagnostics follow the edits", "[algorithms][mesh-fix]")
{
    indexed_triangle_set mesh = make_open_sphere();
    std::swap(
        mesh.indices[face_at(mesh, {0.f, 0.f, -10.f})][1],
        mesh.indices[face_at(mesh, {0.f, 0.f, -10.f})][2]
    );
    append(mesh, its_make_tetrahedron(0.1f), Vec3f{20.f, 0.f, 0.f});
    // Unreferenced vertices (here also the vertices of the removed cap) are removed by the session.
    mesh.vertices.emplace_back(100.f, 100.f, 100.f);

    Session session(std::move(mesh));
    std::vector<bool> referenced(session.mesh().vertices.size(), false);
    for (const stl_triangle_vertex_indices& face : session.mesh().indices)
        for (int vertex_idx : face)
            referenced[vertex_idx] = true;
    CHECK(std::count(referenced.begin(), referenced.end(), false) == 0);
    CHECK(session.diagnostics().holes == 1);
    CHECK(session.diagnostics().flipped_faces == 1);
    CHECK(session.diagnostics().noise_shells == 1);
    CHECK(session.diagnostics().shells == 2);
    CHECK(count_marked(session.flipped()) == 1);
    CHECK(count_marked(session.noise()) == 4);
    CHECK(session.holes().size() == 1);

    CHECK(session.unify_normals() == 1);
    CHECK(session.diagnostics().flipped_faces == 0);
    CHECK(session.remove_noise_shells() == 4);
    CHECK(session.diagnostics().shells == 1);
    CHECK(session.fill_all_holes(HoleFillMode::Smooth) > 0);
    CHECK(session.diagnostics().is_ok());
    CHECK(is_closed_and_oriented(session.mesh()));

    session.detect_intersections();
    CHECK(session.intersections_detected());
    CHECK(session.diagnostics().intersecting_faces == 0);
}

TEST_CASE("Session marking", "[algorithms][mesh-fix]")
{
    indexed_triangle_set mesh = its_make_cube(10., 10., 10.);
    append(mesh, its_make_cube(10., 10., 10.), Vec3f{20.f, 0.f, 0.f});
    Session session(std::move(mesh));
    const MarkParams params;

    CHECK(count_marked(session.region(MarkTool::Triangle, 3, {}, params)) == 1);
    CHECK(count_marked(session.region(MarkTool::Shell, 3, {}, params)) == 12);
    CHECK(count_marked(session.region(MarkTool::Plane, 0, {}, params)) == 2);
    CHECK(count_marked(session.region(MarkTool::Surface, 0, {}, params)) == 2);
    CHECK(count_marked(session.region(MarkTool::Triangle, -1, {}, params)) == 0);

    SECTION("brush stays on the clicked shell")
    {
        MarkParams brush;
        // Touches all triangles at the corner of the first cube and the second cube is far away.
        brush.brush_radius    = 1.f;
        const FaceMask region = session.region(MarkTool::Brush, 0, Vec3f{10.f, 10.f, 0.f}, brush);
        CHECK(count_marked(region) >= 3);
        for (size_t face = 12; face < 24; ++face)
            CHECK(!region[face]);
    }

    SECTION("add, remove, invert, all")
    {
        CHECK(session.mark(session.region(MarkTool::Shell, 0, {}, params), true));
        CHECK(session.marked_count() == 12);
        CHECK(!session.mark(session.region(MarkTool::Triangle, 0, {}, params), true));
        CHECK(session.mark(session.region(MarkTool::Triangle, 0, {}, params), false));
        CHECK(session.marked_count() == 11);
        session.invert_marks();
        CHECK(session.marked_count() == 13);
        session.mark_all();
        CHECK(session.marked_count() == 24);
        session.clear_marks();
        CHECK(session.marked_count() == 0);
    }

    SECTION("grow and shrink")
    {
        session.mark(session.region(MarkTool::Triangle, 0, {}, params), true);
        session.grow_marks();
        CHECK(session.marked_count() > 1);
        CHECK(session.marked_count() <= 12);
        session.shrink_marks();
        CHECK(session.marked_count() < 12);
    }
}

TEST_CASE("Session edits of marked triangles", "[algorithms][mesh-fix]")
{
    indexed_triangle_set mesh = its_make_cube(10., 10., 10.);
    append(mesh, its_make_cube(10., 10., 10.), Vec3f{20.f, 0.f, 0.f});
    Session session(std::move(mesh));
    const MarkParams params;

    SECTION("nothing marked")
    {
        CHECK(session.delete_marked() == 0);
        CHECK(session.flip_marked() == 0);
        CHECK(session.split_marked().indices.empty());
    }
    SECTION("everything marked is not removed")
    {
        session.mark_all();
        CHECK(session.delete_marked() == 0);
        CHECK(session.split_marked().indices.empty());
        CHECK(session.mesh().indices.size() == 24);
    }
    SECTION("delete")
    {
        session.mark(session.region(MarkTool::Plane, 0, {}, params), true);
        CHECK(session.delete_marked() == 2);
        CHECK(session.mesh().indices.size() == 22);
        CHECK(session.marked_count() == 0);
        CHECK(session.diagnostics().holes == 1);
        CHECK(session.diagnostics().open_edges == 4);
    }
    SECTION("flip keeps the marking")
    {
        session.mark(session.region(MarkTool::Shell, 0, {}, params), true);
        CHECK(session.flip_marked() == 12);
        CHECK(session.marked_count() == 12);
        CHECK(session.diagnostics().flipped_faces == 12);
        CHECK(session.unify_normals() == 12);
        CHECK(session.diagnostics().flipped_faces == 0);
    }
    SECTION("flat pieces are not split")
    {
        session.mark(session.region(MarkTool::Triangle, 0, {}, params), true);
        CHECK(session.split_marked().indices.empty());
        session.mark(session.region(MarkTool::Plane, 0, {}, params), true);
        CHECK(session.split_marked().indices.empty());
        CHECK(session.mesh().indices.size() == 24);
        CHECK(session.marked_count() == 2);
    }
    SECTION("split")
    {
        session.mark(session.region(MarkTool::Shell, 12, {}, params), true);
        const indexed_triangle_set part = session.split_marked();
        CHECK(part.indices.size() == 12);
        CHECK(part.vertices.size() == 8);
        CHECK(session.mesh().indices.size() == 12);
        CHECK(session.mesh().vertices.size() == 8);
        CHECK(session.diagnostics().shells == 1);
    }
}

TEST_CASE("Session finds the hole under the cursor", "[algorithms][mesh-fix]")
{
    Session session(make_open_sphere());
    REQUIRE(session.holes().size() == 1);
    const Vec3d down = -Vec3d::UnitZ();

    SECTION("ray through the hole hits the inside of the sphere")
    {
        const int bottom = face_at(session.mesh(), {0.f, 0.f, -10.f});
        CHECK(session.hole_at_ray(Vec3d{0., 0., 30.}, down, bottom, 40.) == 0);
    }
    SECTION("ray missing the mesh")
    {
        CHECK(session.hole_at_ray(Vec3d{0., 0., 30.}, down, -1, 0.) == 0);
        CHECK(session.hole_at_ray(Vec3d{50., 0., 30.}, down, -1, 0.) == -1);
    }
    SECTION("ray hitting the surface far from the hole")
    {
        const int side = face_at(session.mesh(), {10.f, 0.f, 0.f});
        CHECK(session.hole_at_ray(Vec3d{30., 0., 0.}, -Vec3d::UnitX(), side, 20.) == -1);
    }
    SECTION("fill the hole")
    {
        session.mark(session.region(MarkTool::Triangle, 0, {}, {}), true);
        CHECK(session.fill_hole(0, HoleFillMode::Flat) > 0);
        CHECK(session.holes().empty());
        // New triangles are unmarked, the marking of the existing ones is kept.
        CHECK(session.marked_count() == 1);
        CHECK(session.marked()[0]);
        CHECK(session.fill_hole(0, HoleFillMode::Flat) == 0);
    }
}

TEST_CASE("Session auto fix and reset", "[algorithms][mesh-fix]")
{
    indexed_triangle_set mesh = make_open_sphere();
    append(mesh, its_make_tetrahedron(0.1f), Vec3f{20.f, 0.f, 0.f});
    const indexed_triangle_set original = mesh;
    Session session(std::move(mesh));

    const AutoFixReport report = session.auto_fix();
    CHECK(report.holes_filled == 1);
    CHECK(report.noise_faces_removed == 4);
    CHECK(session.diagnostics().is_ok());

    session.mark_all();
    session.reset(original);
    CHECK(session.marked_count() == 0);
    CHECK(session.diagnostics().holes == 1);
    CHECK(session.diagnostics().noise_shells == 1);
}

TEST_CASE("Session never removes the whole mesh", "[algorithms][mesh-fix]")
{
    SECTION("stitching a narrow strip with a large tolerance")
    {
        // Strip 0.5 wide with 0.8 long triangles, all its vertices would be merged.
        indexed_triangle_set strip;
        for (int i = 0; i < 20; ++i) {
            strip.vertices.emplace_back(0.8f * float(i), 0.f, 0.f);
            strip.vertices.emplace_back(0.8f * float(i), 0.5f, 0.f);
        }
        for (int i = 0; i + 1 < 20; ++i) {
            strip.indices.push_back({2 * i, 2 * i + 2, 2 * i + 1});
            strip.indices.push_back({2 * i + 1, 2 * i + 2, 2 * i + 3});
        }
        Session session(std::move(strip));
        CHECK(session.stitch(1.f) == 0);
        CHECK(session.mesh().indices.size() == 38);
        CHECK(session.diagnostics().open_edges == 40);
    }
    SECTION("auto fix of a surface made of pairs of opposite triangles")
    {
        indexed_triangle_set surface;
        surface.vertices = {{0.f, 0.f, 0.f}, {10.f, 0.f, 0.f}, {0.f, 10.f, 0.f}};
        surface.indices  = {{0, 1, 2}, {0, 2, 1}};
        Session session(std::move(surface));
        CHECK(!session.auto_fix().changed());
        CHECK(session.mesh().indices.size() == 2);
    }
}

TEST_CASE("Session brush is a sphere in the transformed mesh", "[algorithms][mesh-fix]")
{
    // Cells of 0.1 x 0.1, stretched four times along X by the transformation.
    Session session(make_grid(10.f, 100));
    MarkParams brush;
    brush.brush_radius = 2.f;
    brush.brush_trafo  = Transform3d::Identity();
    brush.brush_trafo.scale(Vec3d{4., 1., 1.});

    const Vec3f center = Vec3f::Zero();
    const FaceMask region =
        session.region(MarkTool::Brush, face_at(session.mesh(), center), center, brush);
    float max_x = 0.f;
    float max_z = 0.f;
    for (size_t face = 0; face < region.size(); ++face)
        if (region[face]) {
            const Vec3f point = centroid(session.mesh(), face);
            max_x             = std::max(max_x, 4.f * std::abs(point.x()));
            max_z             = std::max(max_z, std::abs(point.z()));
        }
    // A circle of radius 2 in the stretched grid, the size of a cell from it at most.
    CHECK(max_x > 1.6f);
    CHECK(max_x < 2.4f);
    CHECK(max_z > 1.9f);
    CHECK(max_z < 2.1f);
}

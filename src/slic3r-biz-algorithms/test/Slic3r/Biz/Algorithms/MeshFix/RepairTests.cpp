#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <utility>

#include "MeshFixTestUtils.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/Diagnostics.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/Edit.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/Repair.hpp"
#include "Slic3r/Biz/Algorithms/TriangleMesh.hpp"

using namespace Catch;
using namespace Slic3r::Biz::Algorithms::MeshFix;
using namespace Slic3r::Biz::Algorithms::MeshFix::Test;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_cube;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_sphere;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_tetrahedron;
using Slic3r::Biz::Algorithms::TriangleMesh::its_merge_vertices;
using Slic3r::Domain::Vec3f;

namespace {

// Index of the triangle with the extreme centroid coordinate.
size_t extreme_face(const indexed_triangle_set& its, int axis, bool maximum)
{
    size_t best = 0;
    for (size_t face = 1; face < its.indices.size(); ++face) {
        const float value      = centroid(its, face)[axis];
        const float best_value = centroid(its, best)[axis];
        if (maximum ? value > best_value : value < best_value)
            best = face;
    }
    return best;
}

// Sphere with errors far from each other: a flipped triangle at the bottom,
// a degenerate triangle and a duplicate triangle at the side and a noise shell.
indexed_triangle_set make_broken_sphere()
{
    indexed_triangle_set mesh           = its_make_sphere(10., 0.3);
    stl_triangle_vertex_indices& bottom = mesh.indices[extreme_face(mesh, 2, false)];
    std::swap(bottom[1], bottom[2]);
    const stl_triangle_vertex_indices side = mesh.indices[extreme_face(mesh, 0, true)];
    mesh.indices.push_back({side[0], side[1], side[1]});
    const stl_triangle_vertex_indices back = mesh.indices[extreme_face(mesh, 1, true)];
    mesh.indices.push_back(back);
    append(mesh, its_make_tetrahedron(0.1f), Vec3f{20.f, 0.f, 0.f});
    return mesh;
}

} // namespace

TEST_CASE("Delete, flip and extract triangles", "[algorithms][mesh-fix]")
{
    indexed_triangle_set mesh = its_make_cube(1., 1., 1.);
    append(mesh, its_make_cube(1., 1., 1.), Vec3f{5.f, 0.f, 0.f});
    FaceMask second_cube(24, false);
    for (size_t face = 12; face < 24; ++face)
        second_cube[face] = true;

    SECTION("delete")
    {
        CHECK(delete_faces(mesh, second_cube) == 12);
        CHECK(mesh.indices.size() == 12);
        CHECK(mesh.vertices.size() == 8);
        CHECK(is_closed_and_oriented(mesh));
    }
    SECTION("flip")
    {
        CHECK(flip_faces(mesh, second_cube) == 12);
        CHECK(signed_volume(mesh) == Approx(0.).margin(1e-6));
    }
    SECTION("extract")
    {
        const indexed_triangle_set extracted = extract_faces(mesh, second_cube);
        CHECK(extracted.indices.size() == 12);
        CHECK(extracted.vertices.size() == 8);
        CHECK(mesh.indices.size() == 12);
        CHECK(mesh.vertices.size() == 8);
        CHECK(extracted.vertices.front().x() >= 5.f);
        CHECK(signed_volume(extracted) == Approx(1.));
    }
}

TEST_CASE("Unify normals", "[algorithms][mesh-fix]")
{
    indexed_triangle_set sphere = its_make_sphere(10., 0.3);
    const double volume         = signed_volume(sphere);
    std::swap(sphere.indices[3][1], sphere.indices[3][2]);
    std::swap(sphere.indices[42][1], sphere.indices[42][2]);
    CHECK(unify_normals(sphere) == 2);
    CHECK(is_closed_and_oriented(sphere));
    CHECK(signed_volume(sphere) == Approx(volume));

    for (stl_triangle_vertex_indices& face : sphere.indices)
        std::swap(face[1], face[2]);
    CHECK(unify_normals(sphere) == sphere.indices.size());
    CHECK(signed_volume(sphere) == Approx(volume));
}

TEST_CASE("Remove duplicate triangles", "[algorithms][mesh-fix]")
{
    SECTION("equally oriented duplicate")
    {
        indexed_triangle_set cube = its_make_cube(1., 1., 1.);
        cube.indices.push_back(cube.indices[5]);
        // Same triangle, different starting vertex.
        cube.indices.push_back({cube.indices[7][1], cube.indices[7][2], cube.indices[7][0]});
        CHECK(remove_duplicate_faces(cube) == 2);
        CHECK(cube.indices.size() == 12);
        CHECK(is_closed_and_oriented(cube));
    }
    SECTION("inner wall of two merged cubes")
    {
        indexed_triangle_set mesh = its_make_cube(1., 1., 1.);
        // The top of the first cube and the bottom of the second one have the same diagonal.
        append(mesh, its_make_cube(1., 1., 1.), Vec3f{0.f, 0.f, 1.f});
        its_merge_vertices(mesh);
        CHECK(MeshTopology(mesh).count(EdgeKind::NonManifold) > 0);
        CHECK(remove_duplicate_faces(mesh) == 4);
        CHECK(mesh.indices.size() == 20);
        CHECK(MeshTopology(mesh).count(EdgeKind::Open) == 0);
        CHECK(signed_volume(mesh) == Approx(2.));
    }
}

TEST_CASE("Stitch cracks", "[algorithms][mesh-fix]")
{
    indexed_triangle_set cube = its_make_cube(10., 10., 10.);
    // Disconnect two triangles of the vertex 0 by a slightly moved copy of the vertex.
    cube.vertices.push_back(cube.vertices[0] + Vec3f{0.001f, 0.f, 0.f});
    const int copy = int(cube.vertices.size()) - 1;
    for (size_t face : {0, 4})
        for (int& vertex_idx : cube.indices[face])
            if (vertex_idx == 0)
                vertex_idx = copy;
    REQUIRE(MeshTopology(cube).count(EdgeKind::Open) > 0);

    CHECK(stitch_open_edges(cube, 0.0001f) == 0);
    CHECK(stitch_open_edges(cube, 0.01f) == 1);
    CHECK(cube.vertices.size() == 8);
    CHECK(is_closed_and_oriented(cube));
}

TEST_CASE("Diagnose mesh errors", "[algorithms][mesh-fix]")
{
    SECTION("valid mesh")
    {
        const MeshDiagnostics diagnostics = diagnose(its_make_sphere(10., 0.3), {{}, true});
        CHECK(diagnostics.is_ok());
        CHECK(diagnostics.shells == 1);
        CHECK(diagnostics.holes == 0);
    }
    SECTION("broken mesh")
    {
        indexed_triangle_set mesh = make_broken_sphere();
        mesh.indices.erase(mesh.indices.begin() + extreme_face(mesh, 2, true));

        const MeshDiagnostics diagnostics = diagnose(mesh);
        CHECK(!diagnostics.is_ok());
        CHECK(!diagnostics.is_watertight());
        CHECK(diagnostics.holes == 1);
        CHECK(diagnostics.open_edges == 3);
        CHECK(diagnostics.inconsistent_edges == 3);
        CHECK(diagnostics.flipped_faces == 1);
        CHECK(diagnostics.degenerate_faces == 1);
        CHECK(diagnostics.non_manifold_edges == 4);
        CHECK(diagnostics.shells == 2);
        CHECK(diagnostics.noise_shells == 1);
    }
}

TEST_CASE("Mark intersecting triangles", "[algorithms][mesh-fix]")
{
    indexed_triangle_set mesh = its_make_cube(1., 1., 1.);
    CHECK(count_marked(mark_intersecting_faces(mesh)) == 0);

    // A general offset, so that the edges do not cross the triangles exactly at their boundaries.
    append(mesh, its_make_cube(1., 1., 1.), Vec3f{0.5f, 0.3f, 0.4f});
    const FaceMask intersecting = mark_intersecting_faces(mesh);
    CHECK(count_marked(intersecting) > 0);
    // Opposite sides of the cubes do not touch the other cube.
    CHECK(!intersecting[0]);
    CHECK(!intersecting[14]);
}

TEST_CASE("Auto fix", "[algorithms][mesh-fix]")
{
    const double volume       = signed_volume(its_make_sphere(10., 0.3));
    indexed_triangle_set mesh = make_broken_sphere();
    remove_faces_if(mesh, [&](size_t face) { return centroid(mesh, face).z() > 8.f; });

    const AutoFixReport report = auto_fix(mesh);
    CHECK(report.changed());
    CHECK(report.degenerate_faces_removed == 1);
    CHECK(report.duplicate_faces_removed == 1);
    CHECK(report.faces_flipped == 1);
    CHECK(report.noise_faces_removed == 4);
    CHECK(report.holes_filled == 1);
    CHECK(report.faces_added > 0);

    const MeshDiagnostics diagnostics = diagnose(mesh);
    CHECK(diagnostics.is_ok());
    CHECK(diagnostics.shells == 1);
    CHECK(signed_volume(mesh) == Approx(volume).epsilon(0.02));
}

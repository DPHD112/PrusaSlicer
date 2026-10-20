#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <utility>

#include "MeshFixTestUtils.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/Marking.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/MeshTopology.hpp"
#include "Slic3r/Biz/Algorithms/TriangleMesh.hpp"

using namespace Slic3r::Biz::Algorithms::MeshFix;
using namespace Slic3r::Biz::Algorithms::MeshFix::Test;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_cube;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_cylinder;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_sphere;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_tetrahedron;
using Slic3r::Domain::Vec3f;

namespace {

// Cylinder with 36 segments: 72 side triangles, 36 triangles in each cap.
indexed_triangle_set make_cylinder()
{
    return its_make_cylinder(10., 20., 2. * std::numbers::pi / 36.);
}

bool is_side_face(const indexed_triangle_set& its, size_t face)
{
    const stl_triangle_vertex_indices& indices = its.indices[face];
    const Vec3f normal =
        (its.vertices[indices[1]] - its.vertices[indices[0]])
            .cross(its.vertices[indices[2]] - its.vertices[indices[0]]);
    return std::abs(normal.normalized().z()) < 0.1f;
}

} // namespace

TEST_CASE("Topology of a closed mesh", "[algorithms][mesh-fix]")
{
    const indexed_triangle_set cube = its_make_cube(1., 1., 1.);
    const MeshTopology topology(cube);

    CHECK(topology.faces_count() == 12);
    CHECK(topology.edges_count() == 18);
    CHECK(topology.count(EdgeKind::Manifold) == 18);
    CHECK(topology.find_edge(0, 1) >= 0);
    CHECK(topology.find_edge(1, 0) == topology.find_edge(0, 1));
    // Vertices 0 and 6 are opposite corners of the cube.
    CHECK(topology.find_edge(0, 6) < 0);
    for (int face = 0; face < 12; ++face)
        for (int edge = 0; edge < 3; ++edge) {
            const int neighbor = topology.neighbor(face, edge);
            REQUIRE(neighbor >= 0);
            CHECK(neighbor != face);
        }
}

TEST_CASE("Topology classifies broken edges", "[algorithms][mesh-fix]")
{
    SECTION("open edges of a missing triangle")
    {
        indexed_triangle_set cube = its_make_cube(1., 1., 1.);
        cube.indices.pop_back();
        const MeshTopology topology(cube);
        CHECK(topology.count(EdgeKind::Open) == 3);
        CHECK(topology.count(EdgeKind::Manifold) == 15);
    }
    SECTION("inconsistent edges of a flipped triangle")
    {
        indexed_triangle_set cube = its_make_cube(1., 1., 1.);
        std::swap(cube.indices[0][1], cube.indices[0][2]);
        const MeshTopology topology(cube);
        CHECK(topology.count(EdgeKind::Inconsistent) == 3);
        CHECK(topology.count(EdgeKind::Open) == 0);
    }
    SECTION("non-manifold edge shared by three triangles")
    {
        indexed_triangle_set fan;
        fan.vertices =
            {{0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, -1.f, 0.f}, {0.f, 0.f, 1.f}};
        fan.indices = {{0, 1, 2}, {1, 0, 3}, {0, 1, 4}};
        const MeshTopology topology(fan);
        CHECK(topology.count(EdgeKind::NonManifold) == 1);
        CHECK(topology.neighbor(0, 0) == -1);
    }
}

TEST_CASE("Mark shells", "[algorithms][mesh-fix]")
{
    indexed_triangle_set mesh = its_make_cube(1., 1., 1.);
    append(mesh, its_make_cube(1., 1., 1.), Vec3f{5.f, 0.f, 0.f});
    const MeshTopology topology(mesh);

    const Shells shells = find_shells(topology);
    CHECK(shells.count() == 2);
    CHECK(shells.shell_sizes[0] == 12);
    CHECK(shells.shell_sizes[1] == 12);

    const FaceMask shell = mark_shell(topology, 15);
    CHECK(count_marked(shell) == 12);
    CHECK(!shell[0]);
    CHECK(shell[23]);
}

TEST_CASE("Mark surface stops at sharp edges", "[algorithms][mesh-fix]")
{
    const indexed_triangle_set cylinder = make_cylinder();
    const MeshTopology topology(cylinder);
    size_t side_face = 0;
    while (!is_side_face(cylinder, side_face))
        ++side_face;

    const FaceMask surface = mark_surface(cylinder, topology, int(side_face), 30.f);
    CHECK(count_marked(surface) == 72);
    for (size_t face = 0; face < cylinder.indices.size(); ++face)
        CHECK(surface[face] == is_side_face(cylinder, face));

    // Neighbor side triangles differ by 10 degrees at most, a smaller tolerance marks
    // just the triangles of a single segment.
    CHECK(count_marked(mark_surface(cylinder, topology, int(side_face), 5.f)) == 2);
}

TEST_CASE("Mark plane", "[algorithms][mesh-fix]")
{
    SECTION("cube side")
    {
        const indexed_triangle_set cube = its_make_cube(1., 1., 1.);
        const MeshTopology topology(cube);
        CHECK(count_marked(mark_plane(cube, topology, 0, 1.f, 0.001f)) == 2);
    }
    SECTION("cylinder cap")
    {
        const indexed_triangle_set cylinder = make_cylinder();
        const MeshTopology topology(cylinder);
        size_t cap_face = 0;
        while (is_side_face(cylinder, cap_face))
            ++cap_face;
        CHECK(count_marked(mark_plane(cylinder, topology, int(cap_face), 1.f, 0.001f)) == 36);
    }
}

TEST_CASE("Grow and shrink marking", "[algorithms][mesh-fix]")
{
    const indexed_triangle_set cylinder = make_cylinder();
    const MeshTopology topology(cylinder);
    size_t cap_face = 0;
    while (is_side_face(cylinder, cap_face))
        ++cap_face;
    const FaceMask cap = mark_plane(cylinder, topology, int(cap_face), 1.f, 0.001f);

    // All side triangles touch both caps.
    const FaceMask grown = grow_marking(cylinder, cap);
    CHECK(count_marked(grown) == 36 + 72);
    CHECK(shrink_marking(cylinder, grown) == cap);
    // All cap triangles touch the side.
    CHECK(count_marked(shrink_marking(cylinder, cap)) == 0);
}

TEST_CASE("Mark flipped triangles", "[algorithms][mesh-fix]")
{
    SECTION("consistent mesh")
    {
        const indexed_triangle_set sphere = its_make_sphere(10., 0.3);
        CHECK(count_marked(mark_flipped_faces(sphere, MeshTopology(sphere))) == 0);
        const indexed_triangle_set cylinder = make_cylinder();
        CHECK(count_marked(mark_flipped_faces(cylinder, MeshTopology(cylinder))) == 0);
    }
    SECTION("single flipped triangle")
    {
        indexed_triangle_set sphere = its_make_sphere(10., 0.3);
        std::swap(sphere.indices[7][1], sphere.indices[7][2]);
        const FaceMask flipped = mark_flipped_faces(sphere, MeshTopology(sphere));
        CHECK(count_marked(flipped) == 1);
        CHECK(flipped[7]);
    }
    SECTION("inside out mesh")
    {
        indexed_triangle_set cube = its_make_cube(1., 1., 1.);
        for (stl_triangle_vertex_indices& face : cube.indices)
            std::swap(face[1], face[2]);
        CHECK(count_marked(mark_flipped_faces(cube, MeshTopology(cube))) == 12);
    }
    SECTION("open flat surface keeps the majority orientation")
    {
        indexed_triangle_set square;
        square.vertices = {
            {0.f, 0.f, 0.f},
            {1.f, 0.f, 0.f},
            {1.f, 1.f, 0.f},
            {0.f, 1.f, 0.f},
            {2.f, 0.f, 0.f},
            {2.f, 1.f, 0.f}
        };
        square.indices         = {{0, 1, 2}, {0, 2, 3}, {1, 4, 5}, {1, 2, 5}};
        const FaceMask flipped = mark_flipped_faces(square, MeshTopology(square));
        CHECK(count_marked(flipped) == 1);
        CHECK(flipped[3]);
    }
    SECTION("cavity faces inwards")
    {
        indexed_triangle_set block = its_make_cube(10., 10., 10.);
        append(block, inverted(its_make_cube(3., 3., 3.)), Vec3f{0.5f, 3.f, 3.f});
        CHECK(count_marked(mark_flipped_faces(block, MeshTopology(block))) == 0);
        // A flipped triangle of the cavity.
        std::swap(block.indices[15][1], block.indices[15][2]);
        const FaceMask flipped = mark_flipped_faces(block, MeshTopology(block));
        CHECK(count_marked(flipped) == 1);
        CHECK(flipped[15]);
    }
    SECTION("body embedded in a block faces outwards")
    {
        indexed_triangle_set block = its_make_cube(10., 10., 10.);
        append(block, its_make_cube(3., 3., 3.), Vec3f{4.f, 4.f, 4.f});
        CHECK(count_marked(mark_flipped_faces(block, MeshTopology(block))) == 0);
    }
    SECTION("inside out body overlapping a block")
    {
        indexed_triangle_set block = its_make_cube(10., 10., 10.);
        append(block, inverted(its_make_cube(4., 4., 4.)), Vec3f{8.f, 3.f, 3.f});
        const FaceMask flipped = mark_flipped_faces(block, MeshTopology(block));
        CHECK(count_marked(flipped) == 12);
        for (size_t face = 12; face < 24; ++face)
            CHECK(flipped[face]);
    }
}

TEST_CASE("Mark bad edges and noise shells", "[algorithms][mesh-fix]")
{
    indexed_triangle_set mesh = its_make_cube(10., 10., 10.);
    append(mesh, its_make_tetrahedron(0.01f), Vec3f{20.f, 0.f, 0.f});
    const MeshTopology topology(mesh);

    CHECK(count_marked(mark_bad_edge_faces(topology)) == 0);
    const FaceMask noise = mark_noise_shells(mesh, topology);
    CHECK(count_marked(noise) == 4);
    CHECK(noise[12]);

    mesh.indices.erase(mesh.indices.begin());
    const MeshTopology open_topology(mesh);
    // The triangles around the removed one.
    CHECK(count_marked(mark_bad_edge_faces(open_topology)) == 3);
}

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <cmath>
#include <numbers>
#include <set>

#include "MeshFixTestUtils.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/HoleFill.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/MeshTopology.hpp"
#include "Slic3r/Biz/Algorithms/TriangleMesh.hpp"

using namespace Catch;
using namespace Slic3r::Biz::Algorithms::MeshFix;
using namespace Slic3r::Biz::Algorithms::MeshFix::Test;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_cube;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_sphere;
using Slic3r::Domain::Vec3f;

namespace {

constexpr double sphere_radius = 10.;

indexed_triangle_set make_sphere()
{
    return its_make_sphere(sphere_radius, 0.1);
}

// Sphere with the triangles above the given height removed.
indexed_triangle_set make_sphere_with_hole(float height)
{
    indexed_triangle_set sphere = make_sphere();
    remove_faces_if(sphere, [&](size_t face) { return centroid(sphere, face).z() > height; });
    return sphere;
}

std::vector<Hole> holes_of(const indexed_triangle_set& its)
{
    return find_holes(its, MeshTopology(its));
}

} // namespace

TEST_CASE("Find holes", "[algorithms][mesh-fix][hole-fill]")
{
    SECTION("closed mesh has no holes")
    {
        CHECK(holes_of(its_make_cube(1., 1., 1.)).empty());
    }
    SECTION("missing triangle")
    {
        indexed_triangle_set cube = its_make_cube(1., 1., 1.);
        cube.indices.erase(cube.indices.begin());
        const std::vector<Hole> holes = holes_of(cube);
        REQUIRE(holes.size() == 1);
        CHECK(holes[0].size() == 3);
        CHECK(
            std::set<int>(holes[0].vertices.begin(), holes[0].vertices.end())
            == std::set<int>{0, 1, 2}
        );
        CHECK(hole_perimeter(cube, holes[0]) == Approx(2. + std::sqrt(2.)));
        CHECK(find_hole_by_edge(holes, 2, 0) == 0);
        CHECK(find_hole_by_edge(holes, 4, 5) == -1);
    }
    SECTION("missing side of a cube")
    {
        indexed_triangle_set cube = its_make_cube(1., 1., 1.);
        cube.indices.erase(cube.indices.begin(), cube.indices.begin() + 2);
        const std::vector<Hole> holes = holes_of(cube);
        REQUIRE(holes.size() == 1);
        CHECK(holes[0].size() == 4);
    }
    SECTION("two holes touching in a vertex")
    {
        indexed_triangle_set cube = its_make_cube(1., 1., 1.);
        // Triangles {0, 1, 2} and {4, 0, 3} share just the vertex 0.
        remove_faces_if(cube, [](size_t face) { return face == 0 || face == 10; });
        const std::vector<Hole> holes = holes_of(cube);
        REQUIRE(holes.size() == 2);
        CHECK(holes[0].size() == 3);
        CHECK(holes[1].size() == 3);
    }
}

TEST_CASE("Fill planar holes", "[algorithms][mesh-fix][hole-fill]")
{
    const HoleFillMode mode =
        GENERATE(HoleFillMode::Triangulate, HoleFillMode::Flat, HoleFillMode::Smooth);
    indexed_triangle_set cube = its_make_cube(10., 10., 10.);
    // Remove the bottom side.
    cube.indices.erase(cube.indices.begin(), cube.indices.begin() + 2);
    const std::vector<Hole> holes = holes_of(cube);
    REQUIRE(holes.size() == 1);

    const std::vector<int> new_faces = fill_hole(cube, holes[0], {mode});
    CHECK(new_faces.size() >= 2);
    CHECK(is_closed_and_oriented(cube));
    CHECK(signed_volume(cube) == Approx(1000.).epsilon(1e-6));
    // A planar patch stays planar.
    for (const stl_vertex& vertex : cube.vertices)
        CHECK((vertex.z() == Approx(0.).margin(1e-4) || vertex.z() == Approx(10.).margin(1e-4)));
}

TEST_CASE("Fill a hole in a curved surface", "[algorithms][mesh-fix][hole-fill]")
{
    const double sphere_volume   = 4. / 3. * std::numbers::pi * std::pow(sphere_radius, 3.);
    const double original_volume = signed_volume(make_sphere());
    const indexed_triangle_set open_sphere = make_sphere_with_hole(7.f);
    const std::vector<Hole> holes          = holes_of(open_sphere);
    REQUIRE(holes.size() == 1);

    auto fill = [&](HoleFillMode mode)
    {
        indexed_triangle_set sphere = open_sphere;
        fill_hole(sphere, holes[0], {mode});
        CHECK(is_closed_and_oriented(sphere));
        return sphere;
    };

    const indexed_triangle_set triangulated = fill(HoleFillMode::Triangulate);
    const indexed_triangle_set flat         = fill(HoleFillMode::Flat);
    const indexed_triangle_set smooth       = fill(HoleFillMode::Smooth);

    CHECK(triangulated.vertices.size() == open_sphere.vertices.size());
    CHECK(flat.vertices.size() > open_sphere.vertices.size());
    CHECK(smooth.vertices.size() > open_sphere.vertices.size());

    const double flat_error   = std::abs(signed_volume(flat) - original_volume);
    const double smooth_error = std::abs(signed_volume(smooth) - original_volume);
    // The smooth patch restores the spherical cap, unlike the flat one.
    CHECK(smooth_error < 0.25 * flat_error);
    CHECK(signed_volume(smooth) == Approx(sphere_volume).epsilon(0.02));

    // The smooth patch vertices lie close to the sphere, the thin plate patch is a bit flatter
    // than the sphere in its center.
    for (size_t vertex_idx = open_sphere.vertices.size(); vertex_idx < smooth.vertices.size();
         ++vertex_idx)
        CHECK(smooth.vertices[vertex_idx].norm() == Approx(sphere_radius).epsilon(0.04));
}

TEST_CASE("Fill a large hole by the greedy triangulation", "[algorithms][mesh-fix][hole-fill]")
{
    indexed_triangle_set sphere   = make_sphere_with_hole(0.f);
    const std::vector<Hole> holes = holes_of(sphere);
    REQUIRE(holes.size() == 1);
    HoleFillParams params;
    params.max_optimal_triangulation_size = 3;
    params.mode                           = HoleFillMode::Flat;
    CHECK(!fill_hole(sphere, holes[0], params).empty());
    CHECK(is_closed_and_oriented(sphere));
    CHECK(signed_volume(sphere) > 0.);
}

TEST_CASE("Fill multiple holes", "[algorithms][mesh-fix][hole-fill]")
{
    indexed_triangle_set cube = its_make_cube(1., 1., 1.);
    remove_faces_if(cube, [](size_t face) { return face == 0 || face == 10; });
    const std::vector<Hole> holes = holes_of(cube);
    REQUIRE(holes.size() == 2);

    const HoleFillResult result = fill_holes(cube, holes, {HoleFillMode::Triangulate});
    CHECK(result.holes_filled == 2);
    CHECK(result.new_faces.size() == 2);
    CHECK(is_closed_and_oriented(cube));
    CHECK(signed_volume(cube) == Approx(1.));
}

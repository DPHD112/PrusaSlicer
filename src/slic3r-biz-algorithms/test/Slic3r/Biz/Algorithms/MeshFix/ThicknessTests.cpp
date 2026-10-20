#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>
#include <vector>

#include "MeshFixTestUtils.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/Marking.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/MeshTopology.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/Thickness.hpp"
#include "Slic3r/Biz/Algorithms/TriangleMesh.hpp"

using namespace Catch;
using namespace Slic3r::Biz::Algorithms::MeshFix;
using namespace Slic3r::Biz::Algorithms::MeshFix::Test;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_cube;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_cylinder;
using Slic3r::Biz::Algorithms::TriangleMesh::its_make_sphere;
using Slic3r::Domain::Vec3f;

namespace {

Vec3f face_normal(const indexed_triangle_set& its, size_t face)
{
    const stl_triangle_vertex_indices& indices = its.indices[face];
    const Vec3f& a                             = its.vertices[indices[0]];
    return (its.vertices[indices[1]] - a).cross(its.vertices[indices[2]] - a).normalized();
}

// Thickness of the triangles facing the direction.
std::vector<float> thickness_facing(
    const indexed_triangle_set& its,
    const std::vector<float>& thickness,
    const Vec3f& direction
)
{
    std::vector<float> facing;
    for (size_t face = 0; face < its.indices.size(); ++face)
        if (face_normal(its, face).dot(direction) > 0.99f)
            facing.push_back(thickness[face]);
    return facing;
}

void check_all(const std::vector<float>& values, double expected)
{
    REQUIRE(!values.empty());
    for (float value : values)
        CHECK(value == Approx(expected).margin(1e-4));
}

} // namespace

TEST_CASE("Thickness of a plate", "[algorithms][mesh-fix]")
{
    const indexed_triangle_set plate   = its_make_cube(10., 10., 1.);
    const std::vector<float> thickness = measure_thickness(plate, MeshTopology(plate));
    REQUIRE(thickness.size() == plate.indices.size());
    check_all(thickness_facing(plate, thickness, Vec3f::UnitZ()), 1.);
    check_all(thickness_facing(plate, thickness, -Vec3f::UnitZ()), 1.);
    check_all(thickness_facing(plate, thickness, Vec3f::UnitX()), 10.);
    check_all(thickness_facing(plate, thickness, -Vec3f::UnitY()), 10.);
}

TEST_CASE("Thin plate next to a thick block", "[algorithms][mesh-fix]")
{
    indexed_triangle_set mesh = its_make_cube(10., 10., 10.);
    append(mesh, its_make_cube(10., 10., 0.5), Vec3f{20.f, 0.f, 0.f});
    const std::vector<float> thickness = measure_thickness(mesh, MeshTopology(mesh));
    for (size_t face = 0; face < mesh.indices.size(); ++face)
        if (std::abs(face_normal(mesh, face).z()) > 0.99f)
            CHECK(
                thickness[face] == Approx(centroid(mesh, face).x() > 15.f ? 0.5 : 10.).margin(1e-4)
            );
}

TEST_CASE("Thickness of a pin", "[algorithms][mesh-fix]")
{
    const indexed_triangle_set pin     = its_make_cylinder(0.5, 10., 2. * std::numbers::pi / 36.);
    const std::vector<float> thickness = measure_thickness(pin, MeshTopology(pin));
    size_t sides                       = 0;
    for (size_t face = 0; face < pin.indices.size(); ++face) {
        const Vec3f normal = face_normal(pin, face);
        if (std::abs(normal.z()) < 0.01f) {
            // Across the pin through its axis.
            CHECK(thickness[face] == Approx(1.).epsilon(0.01));
            ++sides;
        } else if (std::abs(normal.z()) > 0.99f) {
            // Along the pin.
            CHECK(thickness[face] == Approx(10.).margin(1e-4));
        }
    }
    CHECK(sides >= 72);
}

TEST_CASE("Overlapping shells are measured as one solid", "[algorithms][mesh-fix]")
{
    // The second box covers the half of the cube along X, rays along X pass both.
    indexed_triangle_set mesh = its_make_cube(10., 10., 10.);
    append(mesh, its_make_cube(10., 12., 12.), Vec3f{5.f, -1.f, -1.f});
    const std::vector<float> thickness = measure_thickness(mesh, MeshTopology(mesh));
    for (size_t face = 0; face < 12; ++face)
        if (face_normal(mesh, face).dot(-Vec3f::UnitX()) > 0.99f)
            CHECK(thickness[face] == Approx(15.).margin(1e-4));
    // The side of the box inside of the cube is not a wall.
    for (size_t face = 12; face < 24; ++face)
        if (face_normal(mesh, face).dot(-Vec3f::UnitX()) > 0.99f)
            CHECK(std::isinf(thickness[face]));
}

TEST_CASE("Overlapping bars are as thick as their union", "[algorithms][mesh-fix]")
{
    // A bar of 4 x 2 x 2 made of two bars overlapping by 0.1.
    indexed_triangle_set mesh = its_make_cube(2., 2., 2.);
    append(mesh, its_make_cube(2.1, 2., 2.), Vec3f{1.9f, 0.f, 0.f});
    const std::vector<float> thickness = measure_thickness(mesh, MeshTopology(mesh));
    CHECK(*std::min_element(thickness.begin(), thickness.end()) == Approx(2.).margin(1e-4));
}

TEST_CASE("Shells ending on the same plane", "[algorithms][mesh-fix]")
{
    SECTION("boss on a plate")
    {
        indexed_triangle_set mesh = its_make_cube(10., 10., 2.);
        append(mesh, its_make_cube(6., 6., 5.), Vec3f{2.f, 2.f, 0.f});
        const std::vector<float> thickness = measure_thickness(mesh, MeshTopology(mesh));
        // The bottom and the top of the boss see through the plate and the boss.
        for (size_t face = 12; face < 24; ++face)
            if (std::abs(face_normal(mesh, face).z()) > 0.99f)
                CHECK(thickness[face] == Approx(5.).margin(1e-4));
    }
    SECTION("label on a plate")
    {
        indexed_triangle_set mesh = its_make_cube(10., 10., 0.5);
        append(mesh, its_make_cube(4., 4., 0.8), Vec3f{3.f, 3.f, 0.f});
        const std::vector<float> thickness = measure_thickness(mesh, MeshTopology(mesh));
        for (size_t face = 12; face < 24; ++face)
            if (face_normal(mesh, face).z() > 0.99f)
                CHECK(thickness[face] == Approx(0.8).margin(1e-4));
    }
}

TEST_CASE("Shells inside of other shells are not walls", "[algorithms][mesh-fix]")
{
    // A small cube inside of a block, and a block touching it.
    indexed_triangle_set mesh = its_make_cube(10., 10., 10.);
    append(mesh, its_make_cube(2., 2., 2.), Vec3f{4.f, 4.f, 4.f});
    append(mesh, its_make_cube(10., 10., 10.), Vec3f{10.f, 0.f, 0.f});
    const std::vector<float> thickness = measure_thickness(mesh, MeshTopology(mesh));
    for (size_t face = 12; face < 24; ++face)
        CHECK(std::isinf(thickness[face]));
    for (size_t face = 0; face < mesh.indices.size(); ++face) {
        const float x = centroid(mesh, face).x();
        if (std::abs(x - 10.f) < 1e-3f)
            // The touching sides.
            CHECK(std::isinf(thickness[face]));
        else if (x < 1e-3f)
            // The outer side measures the whole solid.
            CHECK(thickness[face] == Approx(20.).margin(1e-4));
    }
}

TEST_CASE("Thickness of a tiny part", "[algorithms][mesh-fix]")
{
    // Triangles of a few micrometers.
    const indexed_triangle_set sphere  = its_make_sphere(0.025, 0.05);
    const std::vector<float> thickness = measure_thickness(sphere, MeshTopology(sphere));
    for (float value : thickness)
        CHECK(value == Approx(0.05).epsilon(0.02));
}

TEST_CASE("Rays leaving through a hole measure nothing", "[algorithms][mesh-fix]")
{
    indexed_triangle_set plate = its_make_cube(10., 10., 1.);
    remove_faces_if(plate, [&](size_t face) { return face_normal(plate, face).z() < -0.99f; });
    const std::vector<float> thickness = measure_thickness(plate, MeshTopology(plate));
    for (float value : thickness_facing(plate, thickness, Vec3f::UnitZ()))
        CHECK(std::isinf(value));
}

TEST_CASE("Flipped triangles are measured as if they were flipped", "[algorithms][mesh-fix]")
{
    indexed_triangle_set plate = its_make_cube(10., 10., 1.);
    for (size_t face = 0; face < plate.indices.size(); ++face)
        if (face_normal(plate, face).z() > 0.99f)
            std::swap(plate.indices[face][1], plate.indices[face][2]);
    const FaceMask flipped = mark_flipped_faces(plate, MeshTopology(plate));
    REQUIRE(count_marked(flipped) == 2);

    const std::vector<float> thickness = measure_thickness(plate, MeshTopology(plate), flipped);
    for (size_t face = 0; face < plate.indices.size(); ++face)
        if (flipped[face]) {
            CHECK(thickness[face] == Approx(1.).margin(1e-4));
            // Without knowing, the ray goes out of the plate.
            CHECK(std::isinf(measure_thickness(plate, MeshTopology(plate))[face]));
        }
}

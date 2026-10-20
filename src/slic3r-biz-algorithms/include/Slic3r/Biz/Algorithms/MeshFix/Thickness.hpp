#pragma once

#include <admesh/stl.h>

#include <vector>

#include "Slic3r/Biz/Algorithms/MeshFix/Marking.hpp"
#include "Slic3r/Biz/Algorithms/MeshFix/MeshTopology.hpp"

namespace Slic3r::Biz::Algorithms::MeshFix {

/**
 * @brief Wall thickness at each triangle: how far the material reaches behind the triangle.
 *
 * Rays are cast into the solid against the triangle normal, from the centroid and from three
 * points between the centroid and the corners. Each ray measures the distance to where it leaves
 * the solid, overlapping shells count as one solid. The thickness of a triangle is the shortest
 * of its rays, so that a thin spot anywhere on the triangle shows. Points inside of other shells
 * are not on a wall of the solid and are not measured.
 *
 * @param flipped Triangles facing inwards (see mark_flipped_faces()), measured as if they were
 * flipped. Empty if all triangles face outwards.
 * @return Thickness of each triangle in mesh units. Infinity where no ray leaves the solid, which
 * happens behind holes of an open mesh, for triangles inside of other shells and for degenerate
 * triangles.
 */
std::vector<float> measure_thickness(
    const indexed_triangle_set& its,
    const MeshTopology& topology,
    const FaceMask& flipped = {}
);

} // namespace Slic3r::Biz::Algorithms::MeshFix

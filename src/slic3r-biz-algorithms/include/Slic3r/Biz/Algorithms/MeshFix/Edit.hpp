#pragma once

#include <admesh/stl.h>

#include <cstddef>

#include "Slic3r/Biz/Algorithms/MeshFix/MeshTopology.hpp"

/**
 * @brief Manual editing operations applied to marked triangles.
 */
namespace Slic3r::Biz::Algorithms::MeshFix {

/**
 * @brief Remove the marked triangles and the vertices no longer referenced.
 * @return Number of removed triangles.
 */
size_t delete_faces(indexed_triangle_set& its, const FaceMask& mask);

/**
 * @brief Reverse the orientation of the marked triangles.
 * @return Number of flipped triangles.
 */
size_t flip_faces(indexed_triangle_set& its, const FaceMask& mask);

/**
 * @brief Move the marked triangles from the mesh to a new mesh.
 * @return Mesh made of the marked triangles.
 */
indexed_triangle_set extract_faces(indexed_triangle_set& its, const FaceMask& mask);

} // namespace Slic3r::Biz::Algorithms::MeshFix

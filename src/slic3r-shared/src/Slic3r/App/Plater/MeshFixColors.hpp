#pragma once

#include "Slic3r/Domain/Color.hpp"

/**
 * @brief Colors of the mesh fix tool, shared by the 3D view and the legend in the dialog.
 */
namespace Slic3r::App::Plater::MeshFixColors {

inline Domain::ColorRGBA mesh()
{
    return {0.72f, 0.73f, 0.75f, 1.f};
}

inline Domain::ColorRGBA marked()
{
    return {0.15f, 0.70f, 0.35f, 1.f};
}

inline Domain::ColorRGBA preview()
{
    return {0.55f, 0.88f, 0.62f, 1.f};
}

inline Domain::ColorRGBA flipped()
{
    return {0.60f, 0.30f, 0.80f, 1.f};
}

inline Domain::ColorRGBA intersecting()
{
    return {0.95f, 0.50f, 0.10f, 1.f};
}

inline Domain::ColorRGBA noise()
{
    return {0.95f, 0.80f, 0.10f, 1.f};
}

inline Domain::ColorRGBA open_edge()
{
    return {0.95f, 0.10f, 0.10f, 1.f};
}

inline Domain::ColorRGBA non_manifold_edge()
{
    return {1.00f, 0.45f, 0.00f, 1.f};
}

inline Domain::ColorRGBA hole()
{
    return {1.00f, 0.90f, 0.10f, 1.f};
}

inline Domain::ColorRGBA brush()
{
    return {0.15f, 0.70f, 0.35f, 0.25f};
}

} // namespace Slic3r::App::Plater::MeshFixColors

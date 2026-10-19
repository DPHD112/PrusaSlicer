#include "Slic3r/Biz/Algorithms/MeshFix/MeshTopology.hpp"

#include <algorithm>
#include <cassert>

namespace Slic3r::Biz::Algorithms::MeshFix {

namespace {

struct SideEntry
{
    uint64_t key;
    int face;
    int edge;
};

uint64_t edge_key(int vertex_a, int vertex_b)
{
    const auto [v0, v1] = std::minmax(vertex_a, vertex_b);
    return (uint64_t(uint32_t(v0)) << 32) | uint64_t(uint32_t(v1));
}

} // namespace

MeshTopology::MeshTopology(const indexed_triangle_set& its)
{
    const size_t faces_count = its.indices.size();
    m_face_edges.assign(faces_count, Domain::Index3{-1, -1, -1});

    std::vector<SideEntry> sides;
    sides.reserve(3 * faces_count);
    for (size_t face = 0; face < faces_count; ++face) {
        const stl_triangle_vertex_indices& indices = its.indices[face];
        for (int edge = 0; edge < 3; ++edge) {
            const int vertex_a = indices[edge];
            const int vertex_b = indices[(edge + 1) % 3];
            if (vertex_a != vertex_b)
                sides.push_back({edge_key(vertex_a, vertex_b), int(face), edge});
        }
    }
    std::sort(
        sides.begin(),
        sides.end(),
        [](const SideEntry& lhs, const SideEntry& rhs)
        { return lhs.key < rhs.key || (lhs.key == rhs.key && lhs.face < rhs.face); }
    );

    m_edge_faces.reserve(sides.size());
    m_edge_faces_start.reserve(sides.size() / 2 + 2);
    for (size_t begin = 0; begin < sides.size();) {
        size_t end = begin + 1;
        while (end < sides.size() && sides[end].key == sides[begin].key)
            ++end;

        const int edge_index = int(m_edge_vertices.size());
        const int vertex_min = int(sides[begin].key >> 32);
        const int vertex_max = int(sides[begin].key & 0xFFFFFFFFu);
        m_edge_vertices.push_back({vertex_min, vertex_max});
        m_edge_faces_start.push_back(m_edge_faces.size());
        for (size_t i = begin; i < end; ++i) {
            m_face_edges[sides[i].face][sides[i].edge] = edge_index;
            m_edge_faces.push_back({sides[i].face, sides[i].edge});
        }

        EdgeKind kind = EdgeKind::NonManifold;
        if (end - begin == 1) {
            kind = EdgeKind::Open;
        } else if (end - begin == 2) {
            // Consistently oriented neighbors traverse the shared edge in opposite directions.
            const bool first_forward =
                its.indices[sides[begin].face][sides[begin].edge] == vertex_min;
            const bool second_forward =
                its.indices[sides[begin + 1].face][sides[begin + 1].edge] == vertex_min;
            kind = first_forward == second_forward ? EdgeKind::Inconsistent : EdgeKind::Manifold;
        }
        m_edge_kinds.push_back(kind);
        begin = end;
    }
    m_edge_faces_start.push_back(m_edge_faces.size());
}

std::span<const FaceEdge> MeshTopology::edge_faces(int edge_index) const
{
    assert(edge_index >= 0 && size_t(edge_index) < m_edge_vertices.size());
    const size_t begin = m_edge_faces_start[edge_index];
    const size_t end   = m_edge_faces_start[edge_index + 1];
    return {m_edge_faces.data() + begin, end - begin};
}

int MeshTopology::find_edge(int vertex_a, int vertex_b) const
{
    const Domain::Index2 key{std::min(vertex_a, vertex_b), std::max(vertex_a, vertex_b)};
    auto it = std::lower_bound(m_edge_vertices.begin(), m_edge_vertices.end(), key);
    return it != m_edge_vertices.end() && *it == key ? int(it - m_edge_vertices.begin()) : -1;
}

int MeshTopology::neighbor(int face, int edge) const
{
    const int edge_idx = m_face_edges[face][edge];
    if (edge_idx < 0)
        return -1;
    std::span<const FaceEdge> sides = edge_faces(edge_idx);
    if (sides.size() != 2)
        return -1;
    return sides[0].face == face ? sides[1].face : sides[0].face;
}

size_t MeshTopology::count(EdgeKind kind) const
{
    return size_t(std::count(m_edge_kinds.begin(), m_edge_kinds.end(), kind));
}

std::vector<int> mask_to_indices(const FaceMask& mask)
{
    std::vector<int> indices;
    for (size_t i = 0; i < mask.size(); ++i)
        if (mask[i])
            indices.push_back(int(i));
    return indices;
}

} // namespace Slic3r::Biz::Algorithms::MeshFix

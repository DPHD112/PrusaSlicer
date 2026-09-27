// Tree supports "Tree Slim" and "Tree Hybrid", ported from OrcaSlicer's TreeSupport.cpp
// (originally from BambuStudio, which based it on CuraEngine's tree supports).
// OrcaSlicer, BambuStudio and CuraEngine are released under the terms of the AGPLv3 or higher.
//
// The port keeps Orca's node generator: overhang detection with sharp tails and cantilevers,
// contact points sampled on the overhangs, nodes dropped layer by layer while merging along a
// minimum spanning tree and moving out of the avoidance areas, smoothing, and drawing the nodes
// as circles. The resulting areas are handed to the same layer and toolpath code as Organic
// supports. Not ported: independent support layer heights, lightning infill, hole propagation
// for hollow trees (the fill inside wide areas covers that), vertical enforcer points and
// the machine border.

#include "OrcaTreeSupport.hpp"

#include <Slic3r/Log.hpp>
#include <oneapi/tbb/blocked_range.h>
#include <oneapi/tbb/parallel_for.h>
#include <oneapi/tbb/concurrent_unordered_map.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Slic3r/Biz/Algorithms/ExPolygon.hpp"
#include "Slic3r/Biz/Algorithms/BoundingBox.hpp"
#include "Slic3r/Domain/TriangleSelector.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/Support/SupportCommon.hpp"
#include "libslic3r/Support/SupportLayer.hpp"
#include "libslic3r/Support/SupportParameters.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r::FFFSupport;
using namespace Slic3r::Biz;

namespace Slic3r
{

namespace FFFTreeSupport
{

namespace
{

namespace BB     = Biz::Algorithms::BoundingBox;
namespace ExPoly = Biz::Algorithms::ExPolygon;

using coordf_t = double;

constexpr double   TAU                          = 2.0 * M_PI;
constexpr coordf_t MAX_BRANCH_RADIUS            = 10.0;
constexpr coordf_t MIN_BRANCH_RADIUS            = 0.4;
constexpr coordf_t MAX_BRANCH_RADIUS_FIRST_LAYER = 12.0;
constexpr coordf_t MIN_BRANCH_RADIUS_FIRST_LAYER = 2.0;
// Orca's g_config_tree_support_collision_resolution.
constexpr coordf_t RADIUS_SAMPLE_RESOLUTION     = 0.2;
// Hybrid: overhangs larger than this (10 x 10 mm) over free space get straight columns.
const double       THRESH_BIG_OVERHANG          = sqr(scale_(10.));
// Overhangs smaller than 1 mm^2 get no interface.
const double       MINIMUM_ROOF_AREA            = sqr(scale_(1.));

inline double unscale_mm(double v) { return v * SCALING_FACTOR; }
inline double dot_mm(const Point &a, const Point &b) { return unscale_mm(a.x()) * unscale_mm(b.x()) + unscale_mm(a.y()) * unscale_mm(b.y()); }
inline double vsize2_mm(const Point &pt) { return dot_mm(pt, pt); }
inline Point  turn90_ccw(const Point &pt) { return Point(-pt.y(), pt.x()); }
inline Point  scaled_point(const Point &pt, double factor) { return Point(coord_t(std::round(double(pt.x()) * factor)), coord_t(std::round(double(pt.y()) * factor))); }
// Vector of pt's direction with length (in scaled units).
inline Point normal(const Point &pt, double length)
{
    double len = pt.cast<double>().norm();
    if (len < SCALED_EPSILON)
        return pt;
    return scaled_point(pt, length / len);
}

BoundingBox extents(const ExPolygon &expoly) { return ExPoly::get_extents(expoly); }
BoundingBox extents(const ExPolygons &expolys) { return ExPoly::get_extents(expolys); }
Point       bbox_size(const BoundingBox &bbox) { return bbox.defined ? Point(bbox.max - bbox.min) : Point(0, 0); }
double      bbox_radius(const BoundingBox &bbox) { return 0.5 * bbox_size(bbox).cast<double>().norm(); }
Point       bbox_center(const BoundingBox &bbox) { return (bbox.min + bbox.max) / 2; }

bool is_inside_ex(const ExPolygon &polygon, const Point &pt)
{
    return extents(polygon).contains(pt) && ExPoly::contains(polygon, pt);
}

bool is_inside_ex(const ExPolygons &polygons, const Point &pt)
{
    for (const ExPolygon &poly : polygons)
        if (is_inside_ex(poly, pt))
            return true;
    return false;
}

bool overlaps(const ExPolygons &a, const ExPolygons &b)
{
    if (a.empty() || b.empty())
        return false;
    BoundingBox ba = extents(a), bb = extents(b);
    if (! ba.overlap(bb))
        return false;
    return ! intersection_ex(a, b).empty();
}

// Closest point to pt on the segment ab.
Point project_on_segment(const Point &pt, const Point &a, const Point &b)
{
    const Vec2d  ab = (b - a).cast<double>();
    const double l2 = ab.squaredNorm();
    if (l2 <= 0.)
        return a;
    const double t = std::clamp((pt - a).cast<double>().dot(ab) / l2, 0., 1.);
    return a + Point(coord_t(std::round(ab.x() * t)), coord_t(std::round(ab.y() * t)));
}

// Closest point on the polygon outline.
Point project_on_polygon(const Point &pt, const Polygon &poly, double &best_dist2)
{
    Point best = pt;
    for (size_t i = 0; i < poly.points.size(); ++ i) {
        const Point &a = poly.points[i];
        const Point &b = poly.points[(i + 1) % poly.points.size()];
        Point        p = project_on_segment(pt, a, b);
        double       d = (p - pt).cast<double>().squaredNorm();
        if (d < best_dist2) {
            best_dist2 = d;
            best       = p;
        }
    }
    return best;
}

// Closest point on the outlines (contours and holes). Returns pt if there are no outlines.
Point projection_onto(const ExPolygons &polygons, const Point &pt)
{
    Point  best       = pt;
    double best_dist2 = std::numeric_limits<double>::max();
    for (const ExPolygon &expoly : polygons) {
        double d = best_dist2;
        Point  p = project_on_polygon(pt, expoly.contour, d);
        if (d < best_dist2) { best_dist2 = d; best = p; }
        for (const Polygon &hole : expoly.holes) {
            d = best_dist2;
            p = project_on_polygon(pt, hole, d);
            if (d < best_dist2) { best_dist2 = d; best = p; }
        }
    }
    return best;
}

double distance_to_polygon(const Polygon &poly, const Point &pt)
{
    double d2 = std::numeric_limits<double>::max();
    project_on_polygon(pt, poly, d2);
    return std::sqrt(d2);
}

// Move the point out of the polygons by distance (mm) if it is inside them or too close,
// by no more than max_move_distance (mm). Returns false if it can't be done.
bool move_out_expolys(const ExPolygons &polygons, Point &from, double distance, double max_move_distance)
{
    ExPolygons polys_dilated = union_ex(offset_ex(polygons, float(scale_(distance))));
    Point      pt            = projection_onto(polys_dilated, from);
    Point      outward_dir   = pt - from;
    Point      pt_max        = from + normal(outward_dir, scale_(max_move_distance));
    double     dist2         = vsize2_mm(outward_dir);
    if (dist2 > sqr(max_move_distance))
        pt = pt_max;
    if (! is_inside_ex(polys_dilated, from))
        // Already outside and far enough, no need to move.
        return true;
    else if (! is_inside_ex(polygons, from)) {
        // Already outside but not far enough.
        from = pt;
        return true;
    } else if (! is_inside_ex(polygons, pt_max)) {
        from = pt_max;
        return true;
    }
    return false;
}

ExPolygons simplify_expolygons(const ExPolygons &expolys, double tolerance)
{
    ExPolygons out;
    for (const ExPolygon &expoly : expolys)
        append(out, ExPoly::simplify(expoly, tolerance));
    return out;
}

// Keep the largest piece of expoly left after subtracting the avoided region.
ExPolygons avoid_object_remove_extra_small_parts(const ExPolygon &expoly, const ExPolygons &avoid_region)
{
    ExPolygons expolys_out;
    if (expoly.contour.points.empty())
        return expolys_out;
    ExPolygons expolys_avoid = diff_ex(expoly, ClipperUtils::clip_clipper_polygons_with_subject_bbox(avoid_region, extents(expoly)));
    int        idx_max_area  = -1;
    double     max_area      = 0;
    for (int i = 0; i < int(expolys_avoid.size()); ++ i) {
        double a = ExPoly::area(expolys_avoid[i]);
        if (a > max_area) {
            max_area     = a;
            idx_max_area = i;
        }
    }
    if (idx_max_area >= 0)
        expolys_out.emplace_back(std::move(expolys_avoid[idx_max_area]));
    return expolys_out;
}

// Prim's algorithm on the node positions of one part of one layer.
class MinimumSpanningTree
{
public:
    explicit MinimumSpanningTree(const std::vector<Point> &vertices)
    {
        const size_t n = vertices.size();
        if (n == 0)
            return;
        m_adjacent.reserve(n);
        for (const Point &pt : vertices)
            m_adjacent[pt];
        if (n == 1)
            return;
        std::vector<double> smallest_distance(n, std::numeric_limits<double>::max());
        std::vector<size_t> smallest_distance_to(n, 0);
        std::vector<bool>   in_tree(n, false);
        in_tree[0] = true;
        for (size_t i = 1; i < n; ++ i)
            smallest_distance[i] = vsize2_mm(vertices[i] - vertices[0]);
        for (size_t added = 1; added < n; ++ added) {
            // Choose the closest vertex not yet in the tree, breaking ties on coordinates.
            size_t closest = size_t(-1);
            for (size_t i = 0; i < n; ++ i)
                if (! in_tree[i] && (closest == size_t(-1) || smallest_distance[i] < smallest_distance[closest] ||
                    (smallest_distance[i] == smallest_distance[closest] && less(vertices[i], vertices[closest]))))
                    closest = i;
            in_tree[closest] = true;
            const Point &a = vertices[closest];
            const Point &b = vertices[smallest_distance_to[closest]];
            m_adjacent[a].push_back(b);
            m_adjacent[b].push_back(a);
            for (size_t i = 0; i < n; ++ i)
                if (! in_tree[i]) {
                    double d = vsize2_mm(vertices[i] - a);
                    if (d < smallest_distance[i]) {
                        smallest_distance[i]    = d;
                        smallest_distance_to[i] = closest;
                    }
                }
        }
    }

    const std::vector<Point>& adjacent_nodes(const Point &node) const
    {
        static const std::vector<Point> empty;
        auto it = m_adjacent.find(node);
        return it == m_adjacent.end() ? empty : it->second;
    }

private:
    static bool less(const Point &a, const Point &b) { return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y()); }
    std::unordered_map<Point, std::vector<Point>, Domain::PointHash> m_adjacent;
};

enum class NodeType { Circle, Polygon };
enum class OverhangType { Detected, Enforced, SharpTail };

struct SupportNode
{
    // Number of layers to the top of this branch. Negative means a virtual node in the gap
    // between the support and the overhang, which is not printed.
    int          distance_to_top { 0 };
    // Distance to the top contact in mm.
    coordf_t     dist_mm_to_top { 0 };
    Point        position { 0, 0 };
    // Movement towards the neighbour centre or out of the avoidance.
    Point        movement { 0, 0 };
    double       radius { 0 };
    double       max_move_dist { 0 };
    NodeType     type { NodeType::Circle };
    bool         is_corner { false };
    bool         is_processed { false };
    bool         need_extra_wall { false };
    bool         is_sharp_tail { false };
    bool         valid { true };
    // Original overhang area of a polygon node (Tree Hybrid), and of the tips for the roofs.
    ExPolygon    overhang;
    Point        skin_direction { 0, 0 };
    // Number of interface layers still to print below this node.
    int          support_roof_layers_below { 0 };
    int          obj_layer_nr { 0 };
    bool         to_buildplate { true };
    SupportNode *parent { nullptr };
    std::vector<SupportNode*> parents;
    SupportNode *child { nullptr };
    // Neighbours on the same layer that were merged into this node.
    std::list<SupportNode*> merged_neighbours;
    coordf_t     print_z { 0 };
    coordf_t     height { 0 };
};

struct LayerHeightData
{
    coordf_t print_z { 0 };
    coordf_t height { 0 };
    size_t   obj_layer_nr { 0 };
};

struct RadiusLayerPair
{
    coordf_t radius;
    size_t   layer_nr;
    bool operator==(const RadiusLayerPair &rhs) const { return radius == rhs.radius && layer_nr == rhs.layer_nr; }
};

struct RadiusLayerPairHash
{
    size_t operator()(const RadiusLayerPair &elem) const
    {
        return std::hash<coord_t>()(coord_t(elem.radius * 1000.)) ^ std::hash<coord_t>()(coord_t(elem.layer_nr * 7919));
    }
};

// Per object layer result.
struct LayerAreas
{
    ExPolygons base;
    ExPolygons roof;
    ExPolygons roof_base;
    ExPolygons floor;
};

class OrcaTreeSupport
{
public:
    OrcaTreeSupport(const PrintObject &object, bool is_slim, bool is_hybrid, std::function<void()> throw_on_cancel);

    std::vector<LayerAreas> generate();

private:
    const PrintObject     &m_object;
    std::function<void()>  m_throw_on_cancel;
    SupportParameters      m_support_params;
    const bool             m_is_slim;
    const bool             m_is_hybrid;
    size_t                 m_layer_count;

    // Settings.
    bool     m_support_auto;
    int      m_enforce_layers;
    int      m_threshold;
    bool     m_buildplate_only;
    bool     m_dont_support_bridges;
    coordf_t m_extrusion_width;          // mm
    coordf_t m_support_line_width;       // mm
    coordf_t m_xy_distance;              // mm
    coordf_t m_top_z_distance;           // mm
    coordf_t m_bottom_gap;               // mm
    coordf_t m_base_radius;              // mm
    coordf_t m_point_spread;             // mm
    double   m_diameter_angle_scale_factor;
    double   m_branch_angle;             // rad
    size_t   m_roof_layers;
    int      m_top_base_interface_layers;
    size_t   m_bottom_interface_layers;
    coordf_t m_layer_height;             // mm, nominal

    // Per object layer data.
    std::vector<ExPolygons>                m_extrudable;
    std::vector<ExPolygons>                m_sharp_tails;
    std::vector<std::vector<float>>        m_sharp_tails_height;
    std::vector<ExPolygons>                m_cantilevers;
    std::vector<ExPolygons>                m_overhangs;
    std::vector<std::vector<OverhangType>> m_overhang_types;
    size_t                                 m_highest_overhang_layer { 0 };

    // Collision data.
    std::vector<ExPolygons> m_layer_outlines;
    std::vector<ExPolygons> m_layer_outlines_below;
    std::vector<double>     m_max_move_distances;
    mutable tbb::concurrent_unordered_map<RadiusLayerPair, ExPolygons, RadiusLayerPairHash> m_collision_cache;
    mutable tbb::concurrent_unordered_map<RadiusLayerPair, ExPolygons, RadiusLayerPairHash> m_avoidance_cache;

    // Nodes.
    std::mutex                                m_mutex;
    std::deque<std::unique_ptr<SupportNode>>  m_node_storage;
    std::vector<std::vector<SupportNode*>>    m_contact_nodes;
    std::vector<LayerHeightData>              m_layer_heights;
    int                                       m_avg_node_per_layer { 0 };

    void detect_overhangs();
    void generate_contact_points();
    void plan_layer_heights();
    void drop_nodes();
    void smooth_nodes();
    std::vector<LayerAreas> draw_circles();

    SupportNode* create_node(const Point &position, int distance_to_top, int obj_layer_nr, int support_roof_layers_below, bool to_buildplate,
        SupportNode *parent, coordf_t print_z, coordf_t height, coordf_t dist_mm_to_top = 0, coordf_t radius = 0);

    coordf_t ceil_radius(coordf_t radius) const;
    const ExPolygons& get_collision(coordf_t radius, size_t layer_nr) const;
    const ExPolygons& get_avoidance(coordf_t radius, size_t layer_nr) const;
    Polygons get_contours_with_holes(size_t layer_nr) const;

    coordf_t calc_branch_radius(coordf_t base_radius, coordf_t mm_to_top) const;
    coordf_t calc_radius(coordf_t mm_to_top) const { return calc_branch_radius(m_base_radius, mm_to_top); }
    coordf_t get_radius(const SupportNode *node) const
    {
        if (node->radius == 0)
            const_cast<SupportNode*>(node)->radius = calc_radius(node->dist_mm_to_top);
        return node->radius;
    }
};

OrcaTreeSupport::OrcaTreeSupport(const PrintObject &object, bool is_slim, bool is_hybrid, std::function<void()> throw_on_cancel) :
    m_object(object), m_throw_on_cancel(std::move(throw_on_cancel)), m_support_params(object), m_is_slim(is_slim), m_is_hybrid(is_hybrid),
    m_layer_count(object.layer_count())
{
    const PrintObjectConfigView &config         = object.config();
    const SlicingParameters     &slicing_params = object.slicing_parameters();

    m_support_auto         = config.get<Domain::SupportMode>("support_material") == Domain::SupportMode::Everywhere;
    m_enforce_layers       = config.get<int>("support_material_enforce_layers");
    m_threshold            = config.get<int>("support_material_threshold");
    m_buildplate_only      = config.get<bool>("support_material_buildplate_only");
    m_dont_support_bridges = config.get<bool>("dont_support_bridges");
    m_layer_height         = config.get<double>("layer_height");

    double external_perimeter_width = 0.;
    for (size_t region_id = 0; region_id < object.num_printing_regions(); ++ region_id)
        external_perimeter_width = std::max<double>(external_perimeter_width,
            object.printing_region(region_id).flow(object, frExternalPerimeter, m_layer_height).width());
    m_extrusion_width    = external_perimeter_width;
    m_support_line_width = m_support_params.support_material_flow.width();
    m_xy_distance        = config.get<Domain::FloatOrPercentage>("support_material_xy_spacing").get_abs_value(external_perimeter_width);
    m_top_z_distance     = slicing_params.gap_support_object;
    if (m_top_z_distance > EPSILON)
        m_top_z_distance = std::max(m_top_z_distance, slicing_params.min_layer_height);
    m_bottom_gap         = slicing_params.gap_object_support;

    m_base_radius        = std::max(MIN_BRANCH_RADIUS, config.get<double>("support_tree_orca_branch_diameter") / 2.);
    m_point_spread       = std::max(0.5, config.get<double>("support_tree_orca_branch_distance"));
    m_diameter_angle_scale_factor = std::clamp<double>(config.get<double>("support_tree_branch_diameter_angle") * M_PI / 180., 0., 0.5 * M_PI - EPSILON);
    m_branch_angle       = std::clamp<double>(config.get<double>("support_tree_angle") * M_PI / 180., 0., 0.5 * M_PI - EPSILON);
    m_roof_layers        = m_support_params.num_top_interface_layers;
    m_top_base_interface_layers = std::min<int>(int(m_support_params.num_top_base_interface_layers), m_roof_layers > 0 ? int(m_roof_layers) - 1 : 0);
    m_bottom_interface_layers   = m_support_params.num_bottom_interface_layers;

    // Layer outlines used for the collision and avoidance areas.
    m_layer_outlines.assign(m_layer_count, ExPolygons());
    m_layer_outlines_below.assign(m_layer_count, ExPolygons());
    m_max_move_distances.assign(m_layer_count, 0.);
    const double branch_scale_factor = tan(m_branch_angle);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, m_layer_count), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t layer_nr = range.begin(); layer_nr < range.end(); ++ layer_nr) {
            const Layer *layer = m_object.get_layer(int(layer_nr));
            m_max_move_distances[layer_nr] = layer->height * branch_scale_factor;
            m_layer_outlines[layer_nr]     = simplify_expolygons(layer->lslices, scale_(RADIUS_SAMPLE_RESOLUTION));
        }
    });
    for (size_t layer_nr = 0; layer_nr < m_layer_count; ++ layer_nr) {
        m_layer_outlines_below[layer_nr] = layer_nr == 0 ? m_layer_outlines[0] : union_ex(m_layer_outlines_below[layer_nr - 1], m_layer_outlines[layer_nr]);
        m_throw_on_cancel();
    }
}

SupportNode* OrcaTreeSupport::create_node(const Point &position, int distance_to_top, int obj_layer_nr, int support_roof_layers_below, bool to_buildplate,
    SupportNode *parent, coordf_t print_z, coordf_t height, coordf_t dist_mm_to_top, coordf_t radius)
{
    auto node = std::make_unique<SupportNode>();
    node->position                  = position;
    node->distance_to_top           = distance_to_top;
    node->obj_layer_nr              = obj_layer_nr;
    node->support_roof_layers_below = support_roof_layers_below;
    node->to_buildplate             = to_buildplate;
    node->parent                    = parent;
    node->print_z                   = print_z;
    node->height                    = height;
    node->dist_mm_to_top            = dist_mm_to_top;
    node->radius                    = radius;
    SupportNode *raw = node.get();
    std::scoped_lock lock(m_mutex);
    if (parent) {
        raw->parents.push_back(parent);
        raw->type     = parent->type;
        raw->overhang = parent->overhang;
        if (raw->dist_mm_to_top == 0)
            raw->dist_mm_to_top = parent->dist_mm_to_top + parent->height;
        if (raw->radius == 0 && parent->radius > 0)
            raw->radius = parent->radius + (raw->dist_mm_to_top - parent->dist_mm_to_top) * m_diameter_angle_scale_factor;
        parent->child = raw;
        for (SupportNode *neighbor : parent->merged_neighbours) {
            neighbor->child = raw;
            raw->parents.push_back(neighbor);
        }
        raw->is_sharp_tail  = parent->is_sharp_tail;
        raw->skin_direction = parent->skin_direction;
        raw->movement       = position - parent->position;
    }
    m_node_storage.emplace_back(std::move(node));
    return raw;
}

coordf_t OrcaTreeSupport::ceil_radius(coordf_t radius) const
{
    size_t   factor  = size_t(radius / RADIUS_SAMPLE_RESOLUTION);
    coordf_t remains = radius - RADIUS_SAMPLE_RESOLUTION * factor;
    return remains > EPSILON ? radius + RADIUS_SAMPLE_RESOLUTION - remains : radius;
}

const ExPolygons& OrcaTreeSupport::get_collision(coordf_t radius, size_t layer_nr) const
{
    RadiusLayerPair key { ceil_radius(radius), layer_nr };
    if (auto it = m_collision_cache.find(key); it != m_collision_cache.end())
        return it->second;
    ExPolygons collision = offset_ex(m_layer_outlines[layer_nr], float(scale_(key.radius + m_xy_distance)));
    collision = simplify_expolygons(collision, scale_(RADIUS_SAMPLE_RESOLUTION));
    return m_collision_cache.insert({ key, std::move(collision) }).first->second;
}

const ExPolygons& OrcaTreeSupport::get_avoidance(coordf_t radius, size_t layer_nr) const
{
    RadiusLayerPair key { ceil_radius(radius), layer_nr };
    if (auto it = m_avoidance_cache.find(key); it != m_avoidance_cache.end())
        return it->second;
    ExPolygons avoidance;
    if (layer_nr > 0) {
        // Avoidance of a layer depends on all layers below it. Limit the recursion depth by
        // calculating the layer max_recursion_depth below first.
        constexpr size_t max_recursion_depth = 100;
        if (layer_nr >= max_recursion_depth && m_avoidance_cache.find({ key.radius, layer_nr - max_recursion_depth }) == m_avoidance_cache.end())
            get_avoidance(key.radius, layer_nr - max_recursion_depth);
        avoidance = offset_ex(get_avoidance(key.radius, layer_nr - 1), float(- scale_(m_max_move_distances[layer_nr - 1])));
    }
    append(avoidance, get_collision(key.radius, layer_nr));
    avoidance = union_ex(avoidance);
    return m_avoidance_cache.insert({ key, std::move(avoidance) }).first->second;
}

Polygons OrcaTreeSupport::get_contours_with_holes(size_t layer_nr) const
{
    Polygons contours;
    for (const ExPolygon &expoly : m_layer_outlines[layer_nr]) {
        contours.push_back(expoly.contour);
        append(contours, expoly.holes);
    }
    return contours;
}

coordf_t OrcaTreeSupport::calc_branch_radius(coordf_t base_radius, coordf_t mm_to_top) const
{
    // 45 degree tip.
    const coordf_t tip_height = base_radius;
    double radius = mm_to_top > tip_height ? base_radius + (mm_to_top - tip_height) * m_diameter_angle_scale_factor : mm_to_top;
    radius = std::clamp<double>(radius, MIN_BRANCH_RADIUS, MAX_BRANCH_RADIUS);
    // With interface layers the radius should be larger.
    if (m_roof_layers > 0)
        radius = std::max<double>(radius, base_radius);
    return radius;
}

void OrcaTreeSupport::detect_overhangs()
{
    const PrintObjectConfigView &config             = m_object.config();
    const coordf_t               extrusion_width    = m_extrusion_width;
    const coordf_t               extrusion_width_scaled = scale_(extrusion_width);
    const double                 length_thresh_well_supported = scale_(6);
    static const double          sharp_tail_max_support_height = 16.f;
    const double                 enforcer_overhang_offset = scaled<double>(0.8);

    m_extrudable.assign(m_layer_count, ExPolygons());
    m_sharp_tails.assign(m_layer_count, ExPolygons());
    m_sharp_tails_height.assign(m_layer_count, std::vector<float>());
    m_cantilevers.assign(m_layer_count, ExPolygons());
    m_overhangs.assign(m_layer_count, ExPolygons());
    m_overhang_types.assign(m_layer_count, std::vector<OverhangType>());
    m_highest_overhang_layer = 0;

    // Extrudable areas: filter out areas narrower than the extrusion width, without losing detail.
    tbb::parallel_for(tbb::blocked_range<size_t>(0, m_layer_count), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t layer_nr = range.begin(); layer_nr < range.end(); ++ layer_nr) {
            const Layer *layer = m_object.get_layer(int(layer_nr));
            m_extrudable[layer_nr] = intersection_ex(layer->lslices, offset2_ex(layer->lslices, float(- extrusion_width_scaled / 2), float(extrusion_width_scaled)));
        }
    });
    m_throw_on_cancel();

    std::atomic<bool>       detect_sharp_tails { m_support_auto };
    std::atomic<bool>       remove_small_overhangs { m_support_auto };
    std::vector<ExPolygons> overhangs_all_layers(m_layer_count);
    const bool              threshold_auto = m_threshold == 0;
    // +1 makes the threshold inclusive.
    const double            tan_threshold  = threshold_auto ? 0. : tan(M_PI * double(m_threshold + 1) / 180.);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, m_layer_count), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t layer_nr = range.begin(); layer_nr < range.end(); ++ layer_nr) {
            const bool enforced_layer = int(layer_nr) < m_enforce_layers;
            if (! m_support_auto && ! enforced_layer)
                continue;
            const Layer *layer = m_object.get_layer(int(layer_nr));
            if (layer_nr == 0) {
                // Small islands on the first layer are sharp tails.
                for (const ExPolygon &slice : m_extrudable[0]) {
                    Point size = bbox_size(extents(slice));
                    if (! (size.x() > length_thresh_well_supported && size.y() > length_thresh_well_supported)) {
                        m_sharp_tails[0].push_back(slice);
                        m_sharp_tails_height[0].push_back(float(layer->height));
                    }
                }
                continue;
            }
            const Layer *lower_layer = m_object.get_layer(int(layer_nr) - 1);
            coordf_t lower_layer_offset;
            if (enforced_layer)
                lower_layer_offset = -0.15 * extrusion_width;
            else if (threshold_auto)
                lower_layer_offset = 0.5 * extrusion_width;
            else
                lower_layer_offset = lower_layer->height / tan_threshold;
            const ExPolygons &curr_polys  = m_extrudable[layer_nr];
            const ExPolygons &lower_polys = m_extrudable[layer_nr - 1];

            // Normal overhangs.
            ExPolygons lower_layer_offseted = offset_ex(lower_polys, float(scale_(lower_layer_offset)), ClipperLib::jtSquare, 0.);
            overhangs_all_layers[layer_nr]  = diff_ex(curr_polys, lower_layer_offseted);

            if (overhangs_all_layers[layer_nr].size() > 100) {
                detect_sharp_tails     = false;
                remove_small_overhangs = false;
                continue;
            }

            if (m_support_auto && detect_sharp_tails) {
                // A floating region which is not negligible is a sharp tail.
                for (const ExPolygon &expoly : curr_polys)
                    if (! overlaps(offset_ex(expoly, float(0.1 * extrusion_width_scaled)), lower_polys) &&
                        ! offset_ex(expoly, float(-0.1 * extrusion_width_scaled)).empty()) {
                        m_sharp_tails[layer_nr].push_back(expoly);
                        m_sharp_tails_height[layer_nr].push_back(0);
                    }
            }

            // Cantilevers: overhangs whose farthest point is more than 3 mm away from the part below.
            lower_layer_offseted = offset_ex(lower_layer_offseted, float(scale_(std::max(extrusion_width - lower_layer_offset, 0.) + 0.1)));
            for (const ExPolygon &poly : overhangs_all_layers[layer_nr]) {
                Polygons cluster_boundary = ExPoly::to_polygons(intersection_ex(ExPolygons{ poly }, lower_layer_offseted));
                if (cluster_boundary.empty())
                    continue;
                double dist_max = 0;
                for (const Point &pt : poly.contour.points) {
                    double dist_pt = std::numeric_limits<double>::max();
                    for (const Polygon &ply : cluster_boundary)
                        dist_pt = std::min(dist_pt, distance_to_polygon(ply, pt));
                    dist_max = std::max(dist_max, dist_pt);
                }
                if (dist_max > scale_(3))
                    m_cantilevers[layer_nr].emplace_back(poly);
            }
        }
    });
    m_throw_on_cancel();

    // Check whether the sharp tails should be extended higher.
    if (m_support_auto && detect_sharp_tails) {
        for (size_t layer_nr = 1; layer_nr < m_layer_count; ++ layer_nr) {
            const Layer      *layer                         = m_object.get_layer(int(layer_nr));
            const ExPolygons &lower_layer_sharptails        = m_sharp_tails[layer_nr - 1];
            const auto       &lower_layer_sharptails_height = m_sharp_tails_height[layer_nr - 1];
            if (lower_layer_sharptails.empty())
                continue;
            for (const ExPolygon &expoly : m_extrudable[layer_nr]) {
                float accum_height = float(layer->height);
                // If there is no sharp tail below, this is a common region.
                ExPolygons supported_by_lower = intersection_ex(ExPolygons{ expoly }, lower_layer_sharptails);
                if (supported_by_lower.empty())
                    continue;
                // If there is a sharp tail below, check whether it supports this region enough.
                Point size = bbox_size(extents(supported_by_lower));
                if (size.x() > length_thresh_well_supported && size.y() > length_thresh_well_supported)
                    continue;
                // Check whether the sharp tail exceeds the max height.
                for (size_t i = 0; i < lower_layer_sharptails.size(); ++ i)
                    if (ExPoly::overlaps(lower_layer_sharptails[i], expoly)) {
                        accum_height += lower_layer_sharptails_height[i];
                        break;
                    }
                if (accum_height > sharp_tail_max_support_height)
                    continue;
                // If the area grows faster than the threshold, it gets connected to another part
                // or it has a sharp slope and will be supported.
                ExPolygons new_overhang_expolys = diff_ex(ExPolygons{ expoly }, lower_layer_sharptails);
                Point      growth = bbox_size(extents(new_overhang_expolys)) - bbox_size(extents(lower_layer_sharptails));
                if ((growth.x() > scale_(5) && growth.y() > scale_(5)) || ! offset_ex(new_overhang_expolys, float(-5.0 * extrusion_width_scaled)).empty())
                    continue;
                m_sharp_tails[layer_nr].push_back(expoly);
                m_sharp_tails_height[layer_nr].push_back(accum_height);
            }
            m_throw_on_cancel();
        }
    }

    // Group the overhangs into clusters stacked over successive layers, to remove the small ones.
    struct OverhangCluster {
        std::map<int, const ExPolygon*> layer_overhangs;
        ExPolygons  merged_poly;
        BoundingBox merged_bbox;
        int         min_layer { 10000000 };
        int         max_layer { 0 };
        bool        is_cantilever { false };
        bool        is_sharp_tail { false };
        bool        is_small_overhang { false };
        OverhangCluster(const ExPolygon *expoly, int layer_nr, coordf_t offset) {
            layer_overhangs.emplace(layer_nr, expoly);
            ExPolygons dilate1 = offset_ex(*expoly, float(offset));
            merged_poly = union_ex(merged_poly, dilate1);
            min_layer = max_layer = layer_nr;
            merged_bbox = extents(dilate1);
        }
        bool push_back_if_intersects(const ExPolygon &region, int layer_nr, coordf_t offset) {
            if (layer_nr < 1)
                return false;
            auto it = layer_overhangs.find(layer_nr - 1);
            if (it == layer_overhangs.end())
                return false;
            ExPolygons dilate1 = offset_ex(region, float(offset));
            if (dilate1.empty())
                return false;
            BoundingBox bbox = extents(dilate1);
            if (! merged_bbox.overlap(bbox) || ! overlaps(ExPolygons{ *it->second }, dilate1))
                return false;
            layer_overhangs.emplace(layer_nr, &region);
            merged_poly = union_ex(merged_poly, dilate1);
            min_layer   = std::min(min_layer, layer_nr);
            max_layer   = std::max(max_layer, layer_nr);
            merged_bbox = BB::merge(merged_bbox, bbox);
            return true;
        }
    };
    std::vector<OverhangCluster> clusters;
    for (size_t layer_nr = 0; layer_nr < m_layer_count; ++ layer_nr) {
        for (const ExPolygon &overhang : overhangs_all_layers[layer_nr]) {
            OverhangCluster *cluster = nullptr;
            for (OverhangCluster &c : clusters)
                if (c.push_back_if_intersects(overhang, int(layer_nr), extrusion_width_scaled)) {
                    cluster = &c;
                    break;
                }
            if (! cluster)
                cluster = &clusters.emplace_back(&overhang, int(layer_nr), extrusion_width_scaled);
            if (overlaps(ExPolygons{ overhang }, m_cantilevers[layer_nr]))
                cluster->is_cantilever = true;
        }
        m_throw_on_cancel();
    }

    if (m_support_auto && remove_small_overhangs) {
        for (OverhangCluster &cluster : clusters) {
            for (int layer_id = cluster.min_layer; layer_id <= cluster.max_layer; ++ layer_id)
                if (overlaps(m_sharp_tails[layer_id], cluster.merged_poly)) {
                    cluster.is_sharp_tail = true;
                    break;
                }
            if (! cluster.is_sharp_tail && ! cluster.is_cantilever) {
                // Small if the cluster is narrower than 3 extrusion widths.
                Point size = bbox_size(extents(offset_ex(cluster.merged_poly, float(-1 * extrusion_width_scaled))));
                if (size.x() < 2 * extrusion_width_scaled || size.y() < 2 * extrusion_width_scaled)
                    cluster.is_small_overhang = true;
            }
        }
    }
    for (const OverhangCluster &cluster : clusters)
        if (! cluster.is_small_overhang)
            for (const auto &[layer_nr, overhang] : cluster.layer_overhangs)
                m_overhangs[layer_nr].emplace_back(*overhang);

    std::vector<Polygons> enforcers = m_object.slice_support_enforcers();
    std::vector<Polygons> blockers  = m_object.slice_support_blockers();
    m_object.project_and_append_custom_facets(false, Domain::TriangleSelector::TriangleStateType::ENFORCER, enforcers);
    m_object.project_and_append_custom_facets(false, Domain::TriangleSelector::TriangleStateType::BLOCKER, blockers);

    for (size_t layer_nr = 0; layer_nr < m_layer_count; ++ layer_nr) {
        ExPolygons &overhangs = m_overhangs[layer_nr];
        // Add support for every 0.5 mm height of the sharp tails.
        ExPolygons sharp_tail_overhangs;
        if (layer_nr == 0)
            sharp_tail_overhangs = m_sharp_tails[0];
        else {
            ExPolygons lower_layer_expanded = offset_ex(m_extrudable[layer_nr - 1], float(SCALED_EPSILON));
            for (size_t i = 0; i < m_sharp_tails_height[layer_nr].size(); ++ i) {
                ExPolygons areas        = diff_ex(ExPolygons{ m_sharp_tails[layer_nr][i] }, lower_layer_expanded);
                float      accum_height = m_sharp_tails_height[layer_nr][i];
                if (! areas.empty() && int(accum_height * 10) % 5 == 0)
                    append(sharp_tail_overhangs, std::move(areas));
            }
        }
        const bool enforced_layer = int(layer_nr) < m_enforce_layers;
        if (layer_nr < blockers.size() && ! blockers[layer_nr].empty() && ! enforced_layer) {
            ExPolygons blocker   = offset_ex(union_(blockers[layer_nr]), float(scale_(RADIUS_SAMPLE_RESOLUTION)));
            overhangs            = diff_ex(overhangs, blocker);
            m_cantilevers[layer_nr] = diff_ex(m_cantilevers[layer_nr], blocker);
            sharp_tail_overhangs = diff_ex(sharp_tail_overhangs, blocker);
        }
        if (m_dont_support_bridges && ! overhangs.empty() && layer_nr > 0) {
            const Layer &layer       = *m_object.get_layer(int(layer_nr));
            const Layer &lower_layer = *m_object.get_layer(int(layer_nr) - 1);
            Polygons     polys       = ExPoly::to_polygons(overhangs);
            for (const LayerRegion *layerm : layer.regions())
                remove_bridges_from_contacts(m_object.print()->config(), lower_layer, *layerm, float(layerm->flow(frExternalPerimeter).scaled_width()), polys);
            overhangs = union_ex(polys);
        }
        const size_t n_detected = overhangs.size();
        if (layer_nr > 0 && layer_nr < enforcers.size() && ! enforcers[layer_nr].empty()) {
            ExPolygons enforced = intersection_ex(diff_ex(m_extrudable[layer_nr], m_extrudable[layer_nr - 1]), enforcers[layer_nr]);
            if (! enforced.empty()) {
                // Make enforcers work on steep overhangs.
                enforced = diff_ex(offset_ex(enforced, float(enforcer_overhang_offset)), m_extrudable[layer_nr - 1]);
                append(overhangs, std::move(enforced));
            }
        }
        const size_t n_enforced = overhangs.size();
        append(overhangs, std::move(sharp_tail_overhangs));
        auto &types = m_overhang_types[layer_nr];
        for (size_t i = 0; i < overhangs.size(); ++ i)
            types.push_back(i < n_detected ? OverhangType::Detected : i < n_enforced ? OverhangType::Enforced : OverhangType::SharpTail);
        if (! overhangs.empty())
            m_highest_overhang_layer = std::max(m_highest_overhang_layer, layer_nr);
    }
    m_throw_on_cancel();
}

void OrcaTreeSupport::generate_contact_points()
{
    const coordf_t point_spread       = scale_(m_point_spread);
    const coord_t  radius_scaled      = coord_t(scale_(m_base_radius));
    const coordf_t z_distance_top     = m_top_z_distance;
    const int      gap_layers         = z_distance_top == 0 ? 0 : 1;
    const size_t   support_roof_layers = m_roof_layers;

    // Grid points over the entire object, rotated by 22 degrees.
    BoundingBox bounding_box;
    for (const ExPolygons &outlines : m_layer_outlines)
        if (! outlines.empty())
            bounding_box = bounding_box.defined ? BB::merge(bounding_box, extents(outlines)) : extents(outlines);
    m_contact_nodes.assign(m_layer_count, std::vector<SupportNode*>());
    if (! bounding_box.defined)
        return;
    const Point  bounding_box_size = bounding_box.max - bounding_box.min;
    const double rotate_angle      = 22.0 / 180.0 * M_PI;
    const Point  center            = bbox_center(bounding_box);
    const double sin_angle         = std::sin(rotate_angle);
    const double cos_angle         = std::cos(rotate_angle);
    const Vec2d  rotated_dims      = Vec2d(bounding_box_size.x() * cos_angle + bounding_box_size.y() * sin_angle,
                                           bounding_box_size.x() * sin_angle + bounding_box_size.y() * cos_angle) / 2;
    std::vector<Point> grid_points;
    for (double x = - rotated_dims.x(); x < rotated_dims.x(); x += point_spread)
        for (double y = - rotated_dims.y(); y < rotated_dims.y(); y += point_spread) {
            Point pt(coord_t(x * cos_angle - y * sin_angle), coord_t(x * sin_angle + y * cos_angle));
            pt += center;
            if (bounding_box.contains(pt))
                grid_points.push_back(pt);
        }

    // Support must always be at least 1 layer below the overhang.
    const int z_distance_top_layers = int(std::ceil(z_distance_top / m_layer_height - EPSILON)) + 1;
    if (int(m_layer_count) <= z_distance_top_layers + 1)
        return;

    std::atomic<size_t> num_nodes { 0 };
    std::atomic<size_t> nonempty_layers { 0 };
    tbb::parallel_for(tbb::blocked_range<size_t>(1, m_layer_count), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t layer_nr = range.begin(); layer_nr < range.end(); ++ layer_nr) {
            const Layer *layer      = m_object.get_layer(int(layer_nr));
            auto        &curr_nodes = m_contact_nodes[layer_nr - 1];
            const coordf_t bottom_z = layer->bottom_z();
            std::unordered_set<Point, Domain::PointHash> already_inserted;
            bool is_sharp_tail = false;

            // The contact node is a virtual node which is the invisible gap between the support and the object:
            // it is placed at the bottom of the overhang layer, its height is the gap distance.
            auto insert_point = [&](const Point &pt, const ExPolygon &overhang, double radius, bool force_add, bool add_interface) -> SupportNode* {
                Point hash_pos = pt / (radius_scaled + 1);
                if (! force_add && already_inserted.count(hash_pos))
                    return nullptr;
                already_inserted.emplace(hash_pos);
                SupportNode *contact_node = create_node(pt, -gap_layers, int(layer_nr) - 1, add_interface ? int(support_roof_layers) : 0, true, nullptr,
                    bottom_z, z_distance_top, 0, radius);
                contact_node->overhang      = overhang;
                contact_node->is_sharp_tail = is_sharp_tail;
                curr_nodes.emplace_back(contact_node);
                return contact_node;
            };

            const ExPolygons &overhangs = m_overhangs[layer_nr];
            for (size_t overhang_idx = 0; overhang_idx < overhangs.size(); ++ overhang_idx) {
                const ExPolygon &overhang_part = overhangs[overhang_idx];
                is_sharp_tail = m_overhang_types[layer_nr][overhang_idx] == OverhangType::SharpTail;
                ExPolygons overhangs_regular;
                if (m_is_hybrid && ExPoly::area(overhang_part) > THRESH_BIG_OVERHANG && ! is_sharp_tail) {
                    // The part of a big overhang above the object gets tree branches, the part over
                    // free space gets a straight column (polygon node).
                    overhangs_regular           = offset_ex(intersection_ex(ExPolygons{ overhang_part }, m_layer_outlines_below[layer_nr - 1]), float(radius_scaled));
                    ExPolygons overhangs_normal = diff_ex(ExPolygons{ overhang_part }, overhangs_regular);
                    if (ExPoly::area(overhangs_normal) > THRESH_BIG_OVERHANG) {
                        for (const ExPolygon &overhang : overhangs_normal) {
                            BoundingBox  overhang_bounds = extents(overhang);
                            SupportNode *contact_node    = insert_point(bbox_center(overhang_bounds), overhang, unscale_mm(bbox_radius(overhang_bounds)), true, true);
                            contact_node->type = NodeType::Polygon;
                        }
                    } else
                        overhangs_regular = ExPolygons{ overhang_part };
                } else
                    overhangs_regular = ExPolygons{ overhang_part };

                for (const ExPolygon &overhang : overhangs_regular) {
                    const bool  add_interface   = ExPoly::area(overhang) > MINIMUM_ROOF_AREA && ! is_sharp_tail;
                    BoundingBox overhang_bounds = extents(overhang);
                    const double radius         = std::clamp(unscale_mm(bbox_radius(overhang_bounds)), MIN_BRANCH_RADIUS, m_base_radius);
                    // Add supports at the corners sharper than 135 degrees.
                    const Points &points = overhang.contour.points;
                    const int     n      = int(points.size());
                    for (int i = 0; i < n; ++ i) {
                        const Point &pt = points[i];
                        Vec2d v1 = (pt - points[(i - 1 + n) % n]).cast<double>().normalized();
                        Vec2d v2 = (pt - points[(i + 1) % n]).cast<double>().normalized();
                        if (v1.dot(v2) > -0.7)
                            if (SupportNode *node = insert_point(pt, overhang, radius, false, add_interface); node)
                                node->is_corner = true;
                    }
                    // Add supports along the contours.
                    auto sample_polygon = [&](const Polygon &polygon) {
                        const Points &pts = polygon.points;
                        if (pts.empty())
                            return;
                        double next = 0., walked = 0.;
                        for (size_t i = 0; i < pts.size(); ++ i) {
                            const Point &a   = pts[i];
                            const Point &b   = pts[(i + 1) % pts.size()];
                            const Vec2d  ab  = (b - a).cast<double>();
                            const double len = ab.norm();
                            while (next <= walked + len) {
                                const double t = len > 0 ? (next - walked) / len : 0.;
                                insert_point(a + Point(coord_t(ab.x() * t), coord_t(ab.y() * t)), overhang, radius, false, add_interface);
                                next += point_spread;
                            }
                            walked += len;
                        }
                    };
                    sample_polygon(overhang.contour);
                    for (const Polygon &hole : overhang.holes)
                        sample_polygon(hole);
                    // No inner supports for sharp tails.
                    if (is_sharp_tail)
                        continue;
                    // Add inner supports on the grid.
                    ExPolygons overhang_inner = offset_ex(overhang, float(- radius_scaled));
                    for (const Point &candidate : grid_points)
                        if (overhang_bounds.contains(candidate) && is_inside_ex(overhang_inner, candidate))
                            insert_point(candidate, overhang, radius, false, add_interface);
                }
            }
            if (! curr_nodes.empty()) {
                ++ nonempty_layers;
                num_nodes += curr_nodes.size();
            }
            m_throw_on_cancel();
        }
    });
    if (nonempty_layers > 0)
        m_avg_node_per_layer = int(num_nodes / nonempty_layers);
}

void OrcaTreeSupport::plan_layer_heights()
{
    // Support layers follow the object layers.
    m_layer_heights.assign(m_layer_count, LayerHeightData());
    for (size_t layer_nr = 0; layer_nr < m_layer_count; ++ layer_nr) {
        const Layer *layer = m_object.get_layer(int(layer_nr));
        m_layer_heights[layer_nr] = { layer->print_z, layer->height, layer_nr };
    }
    // Adjust the contact nodes' distance_to_top to the layer heights. With a large top Z distance,
    // one gap layer is not enough, the gap is split into multiple layers.
    for (size_t layer_nr = 0; layer_nr < m_contact_nodes.size(); ++ layer_nr) {
        if (m_contact_nodes[layer_nr].empty())
            continue;
        SupportNode   *node1      = m_contact_nodes[layer_nr].front();
        const coordf_t new_height = m_layer_heights[layer_nr].height;
        if (std::abs(node1->height - new_height) < EPSILON)
            continue;
        if (m_top_z_distance < EPSILON && node1->height < EPSILON)
            // Zero top Z distance, soluble interface.
            continue;
        coordf_t accum_height = 0;
        int      num_layers   = 0;
        for (int i = int(layer_nr); i >= 0; -- i)
            if (m_layer_heights[i].height > EPSILON) {
                accum_height += m_layer_heights[i].height;
                ++ num_layers;
                if (accum_height > node1->height - EPSILON)
                    break;
            }
        for (SupportNode *node : m_contact_nodes[layer_nr]) {
            node->height          = new_height;
            node->distance_to_top = - num_layers;
        }
    }
}

void OrcaTreeSupport::drop_nodes()
{
    const coordf_t support_extrusion_width = m_support_line_width;
    const double   tan_angle               = tan(m_branch_angle);
    // When nodes are thick, they can move further. This is the max angle.
    const coordf_t max_move_distance       = tan_angle * m_layer_height;
    const double   max_move_distance2      = max_move_distance * max_move_distance;
    const bool     support_on_buildplate_only = m_buildplate_only;
    // Do not move contact points under 5 mm.
    const float    DO_NOT_MOVE_UNDER_MM    = m_is_slim ? 0.f : 5.f;

    auto get_max_move_dist = [this, tan_angle, support_extrusion_width](const SupportNode *node, int power = 1) {
        if (node->max_move_dist == 0) {
            const_cast<SupportNode*>(node)->radius        = get_radius(node);
            const_cast<SupportNode*>(node)->max_move_dist = std::min(tan_angle * node->height, support_extrusion_width);
        }
        double move_dist = node->max_move_dist;
        return power == 2 ? sqr(move_dist) : move_dist;
    };

    std::vector<LayerHeightData> &layer_heights = m_layer_heights;
    if (layer_heights.empty() || m_contact_nodes.empty())
        return;

    {
        // Precalculate the avoidance of all possible radii in parallel.
        std::vector<std::set<coordf_t>> all_layer_radius(m_contact_nodes.size());
        std::vector<std::set<coordf_t>> all_layer_node_dist(m_contact_nodes.size());
        for (size_t layer_nr = m_contact_nodes.size() - 1; layer_nr > 0; -- layer_nr) {
            auto &layer_radius    = all_layer_radius[layer_nr];
            auto &layer_node_dist = all_layer_node_dist[layer_nr];
            for (const SupportNode *p_node : m_contact_nodes[layer_nr])
                layer_node_dist.emplace(p_node->dist_mm_to_top);
            size_t layer_nr_next = layer_nr - 1;
            if (layer_nr_next > 0)
                for (coordf_t node_dist : layer_node_dist)
                    all_layer_node_dist[layer_nr_next].emplace(node_dist + layer_heights[layer_nr].height);
            for (coordf_t node_dist : layer_node_dist)
                layer_radius.emplace(calc_radius(node_dist));
        }
        tbb::parallel_for(tbb::blocked_range<size_t>(0, m_contact_nodes.size() - 1), [&](const tbb::blocked_range<size_t> &range) {
            for (size_t layer_nr = range.begin(); layer_nr < range.end(); ++ layer_nr)
                for (coordf_t node_radius : all_layer_radius[layer_nr]) {
                    size_t obj_layer_nr = layer_heights[layer_nr].obj_layer_nr;
                    get_avoidance(node_radius, obj_layer_nr);
                    get_collision(0, obj_layer_nr);
                    get_collision(node_radius, obj_layer_nr);
                }
        });
        m_throw_on_cancel();
    }

    // Skip layer 0, the nodes can't be dropped any further.
    for (size_t layer_nr = m_contact_nodes.size() - 1; layer_nr > 0; -- layer_nr) {
        m_throw_on_cancel();
        auto &layer_contact_nodes = m_contact_nodes[layer_nr];
        if (layer_contact_nodes.empty())
            continue;

        const size_t   layer_nr_next     = layer_nr - 1;
        const coordf_t print_z_next      = layer_heights[layer_nr_next].print_z;
        const coordf_t height_next       = layer_heights[layer_nr_next].height;
        const size_t   obj_layer_nr      = layer_heights[layer_nr].obj_layer_nr;
        const size_t   obj_layer_nr_next = layer_heights[layer_nr_next].obj_layer_nr;

        // Leaves on this layer that would result in unsupported (mid-air) branches.
        std::deque<std::pair<size_t, SupportNode*>> unsupported_branch_leaves;

        const Polygons layer_contours = get_contours_with_holes(obj_layer_nr);
        std::mutex     line_cache_mutex;
        std::map<std::pair<std::pair<coord_t, coord_t>, std::pair<coord_t, coord_t>>, bool> line_cache;
        auto is_line_cut_by_contour = [&](const Point &a, const Point &b) {
            auto key = std::make_pair(std::make_pair(a.x(), a.y()), std::make_pair(b.x(), b.y()));
            {
                std::scoped_lock lock(line_cache_mutex);
                if (auto it = line_cache.find(key); it != line_cache.end())
                    return it->second;
            }
            bool cut = ! intersection_ln(Line(b, a), layer_contours).empty();
            std::scoped_lock lock(line_cache_mutex);
            line_cache[key] = cut;
            line_cache[std::make_pair(key.second, key.first)] = cut;
            return cut;
        };

        // Group together all nodes of each part. Nodes outside all parts go to the 0th group.
        const ExPolygons &parts = m_layer_outlines_below[obj_layer_nr];
        std::vector<std::unordered_map<Point, SupportNode*, Domain::PointHash>> nodes_per_part(1 + parts.size());
        for (SupportNode *p_node : layer_contact_nodes) {
            const SupportNode &node = *p_node;
            if (support_on_buildplate_only && ! node.to_buildplate) {
                // Can't rest on the model and can't reach the build plate, drop the node.
                unsupported_branch_leaves.push_front({ layer_nr, p_node });
                continue;
            }
            if (node.to_buildplate || parts.empty()) {
                nodes_per_part[0][node.position] = p_node;
                continue;
            }
            // The node belongs to the part it is inside of, or to the closest part.
            coordf_t closest_part_distance2 = std::numeric_limits<coordf_t>::max();
            size_t   closest_part           = 0;
            for (size_t part_index = 0; part_index < parts.size(); ++ part_index) {
                if (is_inside_ex(parts[part_index], node.position)) {
                    closest_part = part_index;
                    break;
                }
                double d2 = std::numeric_limits<double>::max();
                Point  closest_point = project_on_polygon(node.position, parts[part_index].contour, d2);
                const coordf_t distance2 = vsize2_mm(node.position - closest_point);
                if (distance2 < closest_part_distance2) {
                    closest_part_distance2 = distance2;
                    closest_part           = part_index;
                }
            }
            nodes_per_part[closest_part + 1][node.position] = p_node;
        }

        // A minimum spanning tree for every part.
        std::vector<MinimumSpanningTree> spanning_trees;
        spanning_trees.reserve(nodes_per_part.size());
        for (const auto &group : nodes_per_part) {
            std::vector<Point> points;
            points.reserve(group.size());
            for (const auto &entry : group)
                points.emplace_back(entry.first);
            // Sort for a deterministic tree.
            std::sort(points.begin(), points.end(), [](const Point &a, const Point &b) { return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y()); });
            spanning_trees.emplace_back(points);
        }

        for (size_t group_index = 0; group_index < nodes_per_part.size(); ++ group_index) {
            auto                      &nodes_this_part = nodes_per_part[group_index];
            const MinimumSpanningTree &mst             = spanning_trees[group_index];
            std::vector<std::pair<Point, SupportNode*>> nodes_vec(nodes_this_part.begin(), nodes_this_part.end());
            std::sort(nodes_vec.begin(), nodes_vec.end(), [](const auto &a, const auto &b) { return a.first.x() < b.first.x() || (a.first.x() == b.first.x() && a.first.y() < b.first.y()); });

            // First pass: merge all nodes that are close together. Sequential, as the nodes merge
            // into and invalidate each other in place.
            for (auto &entry : nodes_vec) {
                SupportNode *p_node = entry.second;
                SupportNode &node   = *p_node;
                if (! p_node->valid)
                    continue;
                const std::vector<Point> &neighbours = mst.adjacent_nodes(node.position);
                if (node.type == NodeType::Polygon) {
                    // Merge all circle neighbours that are completely inside the polygon into this node.
                    for (const Point &neighbour : neighbours) {
                        SupportNode *neighbour_node = nodes_this_part[neighbour];
                        if (! neighbour_node->valid || neighbour_node->type == NodeType::Polygon)
                            continue;
                        const coord_t r = coord_t(scale_(neighbour_node->radius));
                        if (is_inside_ex(node.overhang, neighbour) &&
                            is_inside_ex(node.overhang, neighbour + Point(0, r)) && is_inside_ex(node.overhang, neighbour - Point(0, r)) &&
                            is_inside_ex(node.overhang, neighbour - Point(r, 0)) && is_inside_ex(node.overhang, neighbour + Point(r, 0))) {
                            node.distance_to_top           = std::max(node.distance_to_top, neighbour_node->distance_to_top);
                            node.support_roof_layers_below = std::max(node.support_roof_layers_below, neighbour_node->support_roof_layers_below);
                            node.dist_mm_to_top            = std::max(node.dist_mm_to_top, neighbour_node->dist_mm_to_top);
                            node.merged_neighbours.push_front(neighbour_node);
                            node.merged_neighbours.insert(node.merged_neighbours.end(), neighbour_node->merged_neighbours.begin(), neighbour_node->merged_neighbours.end());
                            neighbour_node->valid = false;
                        }
                    }
                } else if (neighbours.size() == 1 && vsize2_mm(neighbours[0] - node.position) < get_max_move_dist(p_node, 2) &&
                           mst.adjacent_nodes(neighbours[0]).size() == 1 && nodes_this_part[neighbours[0]]->type != NodeType::Polygon) {
                    // Just two nodes left, very close, and the only neighbour is not a polygon node:
                    // insert a new node between them and let both original nodes fade.
                    Point      next_position = (node.position + neighbours[0]) / 2;
                    coordf_t   next_radius   = calc_radius(node.dist_mm_to_top + height_next);
                    if (group_index == 0) {
                        // Avoid collisions.
                        const coordf_t max_move_between_samples = max_move_distance + RADIUS_SAMPLE_RESOLUTION + EPSILON;
                        move_out_expolys(get_avoidance(next_radius, obj_layer_nr_next), next_position, RADIUS_SAMPLE_RESOLUTION + EPSILON, max_move_between_samples);
                    }
                    SupportNode *neighbour = nodes_this_part[neighbours[0]];
                    SupportNode *node_parent;
                    if (p_node->parent && neighbour->parent)
                        node_parent = (node.dist_mm_to_top >= neighbour->dist_mm_to_top) ? p_node : neighbour;
                    else
                        node_parent = p_node->parent ? p_node : neighbour;
                    // Make sure the next pass doesn't drop down either of these (since that already happened).
                    node_parent->merged_neighbours.push_front(node_parent == p_node ? neighbour : p_node);
                    const bool   to_buildplate = ! is_inside_ex(get_collision(0, obj_layer_nr_next), next_position);
                    SupportNode *next_node     = create_node(next_position, node_parent->distance_to_top + 1, int(obj_layer_nr_next),
                        node_parent->support_roof_layers_below - (node_parent->distance_to_top >= 0 ? 1 : 0), to_buildplate, node_parent, print_z_next, height_next);
                    get_max_move_dist(next_node);
                    m_contact_nodes[layer_nr_next].push_back(next_node);
                    neighbour->valid = false;
                    p_node->valid    = false;
                } else if (neighbours.size() > 1) {
                    // Merge all neighbours that are too close into this node. Leaf nodes are not merged,
                    // as they would then move more than the maximum move distance.
                    for (const Point &neighbour : neighbours)
                        if (vsize2_mm(neighbour - node.position) < get_max_move_dist(&node, 2)) {
                            SupportNode *neighbour_node = nodes_this_part[neighbour];
                            if (neighbour_node->type == NodeType::Polygon)
                                continue;
                            // Only bigger nodes merge smaller nodes.
                            if (node.dist_mm_to_top < neighbour_node->dist_mm_to_top)
                                continue;
                            if (p_node->valid) {
                                node.merged_neighbours.push_front(neighbour_node);
                                node.merged_neighbours.insert(node.merged_neighbours.end(), neighbour_node->merged_neighbours.begin(), neighbour_node->merged_neighbours.end());
                                neighbour_node->valid = false;
                            }
                        }
                }
            }

            // Second pass: move all middle nodes. Parallel, the side effects are recorded per node
            // and applied afterwards in node order.
            struct PendingNode {
                Point        position;
                int          distance_to_top           = 0;
                int          support_roof_layers_below = 0;
                bool         to_buildplate             = false;
                SupportNode *parent                    = nullptr;
                bool         zero_max_move             = false;
                bool         has_overhang              = false;
                ExPolygon    overhang;
                bool         clamp_radius              = false;
                coordf_t     parent_radius             = 0;
                double       dist_to_outer             = 0;
            };
            struct PassTwoResult {
                bool                     invalidate       = false;
                bool                     unsupported_leaf = false;
                std::vector<PendingNode> pending;
            };
            std::vector<PassTwoResult> pass2_results(nodes_vec.size());
            tbb::parallel_for(tbb::blocked_range<size_t>(0, nodes_vec.size()), [&](const tbb::blocked_range<size_t> &node_range) {
                for (size_t node_idx = node_range.begin(); node_idx < node_range.end(); ++ node_idx) {
                    PassTwoResult     &out    = pass2_results[node_idx];
                    SupportNode       *p_node = nodes_vec[node_idx].second;
                    const SupportNode &node   = *p_node;
                    if (! p_node->valid)
                        continue;
                    if (node.type == NodeType::Polygon) {
                        // Polygon nodes do not merge or move, keep only the part that is not removed by the next layer.
                        ExPolygons overhangs_next = diff_ex(ExPolygons{ node.overhang }, get_collision(0, obj_layer_nr_next));
                        for (ExPolygon &overhang : overhangs_next) {
                            PendingNode pending;
                            pending.position                  = overhang.contour.centroid();
                            pending.distance_to_top           = node.distance_to_top + 1;
                            pending.support_roof_layers_below = node.support_roof_layers_below - (node.distance_to_top >= 0 ? 1 : 0);
                            pending.to_buildplate             = true;
                            pending.parent                    = p_node;
                            pending.zero_max_move             = true;
                            pending.has_overhang              = true;
                            pending.overhang                  = std::move(overhang);
                            out.pending.emplace_back(std::move(pending));
                        }
                        continue;
                    }

                    // If the branch falls completely inside a collision area (the entire branch would be
                    // removed by the X/Y offset), delete it.
                    if (group_index > 0 && is_inside_ex(get_collision(0, obj_layer_nr), node.position)) {
                        const coordf_t branch_radius_node = get_radius(p_node);
                        Point          to_outside         = projection_onto(get_collision(0, obj_layer_nr), node.position);
                        double         dist2_to_outside   = vsize2_mm(node.position - to_outside);
                        if (dist2_to_outside >= branch_radius_node * branch_radius_node) {
                            // Too far inside.
                            if (support_on_buildplate_only)
                                out.unsupported_leaf = true;
                            else
                                out.invalidate = true;
                            continue;
                        }
                        // If the link between the parent and this node is cut by the contours, this node is a bottom contact.
                        if (p_node->parent && ! intersection_ln(Line(p_node->position, p_node->parent->position), layer_contours).empty()) {
                            out.invalidate = true;
                            continue;
                        }
                    }
                    Point next_layer_vertex = node.position;
                    Point move_to_neighbor_center(0, 0);
                    const std::vector<Point> &neighbours = mst.adjacent_nodes(node.position);
                    // Don't merge neighbours under 5 mm; only merge a node with a single neighbour further than the max move distance.
                    const double dist2_to_first_neighbor = neighbours.empty() ? 0 : vsize2_mm(neighbours[0] - node.position);
                    if (node.print_z > DO_NOT_MOVE_UNDER_MM &&
                        (neighbours.size() > 1 || (neighbours.size() == 1 && dist2_to_first_neighbor >= get_max_move_dist(p_node, 2)))) {
                        // Move towards the average position of all neighbours.
                        Vec2d sum_direction(0, 0);
                        for (const Point &neighbour : neighbours) {
                            // Do not move to a neighbour to be deleted.
                            SupportNode *neighbour_node = nodes_this_part.at(neighbour);
                            if (! neighbour_node->valid)
                                continue;
                            Point  direction          = neighbour - node.position;
                            double dist2_to_neighbor  = vsize2_mm(direction);
                            // Do not move to a neighbour too far away to converge before reaching the bed.
                            coordf_t branch_bottom_radius    = calc_radius(node.dist_mm_to_top + node.print_z);
                            coordf_t neighbour_bottom_radius = calc_radius(neighbour_node->dist_mm_to_top + neighbour_node->print_z);
                            double   max_converge_distance   = tan_angle * (p_node->print_z - DO_NOT_MOVE_UNDER_MM) + std::max(branch_bottom_radius, neighbour_bottom_radius);
                            if (dist2_to_neighbor > max_converge_distance * max_converge_distance)
                                continue;
                            if (is_line_cut_by_contour(node.position, neighbour))
                                continue;
                            sum_direction += direction.cast<double>() * (1. / dist2_to_neighbor);
                        }
                        move_to_neighbor_center = Point(coord_t(sum_direction.x()), coord_t(sum_direction.y()));
                    }

                    const coordf_t    next_radius    = calc_radius(node.dist_mm_to_top + height_next);
                    const ExPolygons &avoidance_next = get_avoidance(next_radius, obj_layer_nr_next);
                    Point  to_outside         = projection_onto(avoidance_next, node.position);
                    Point  direction_to_outer = to_outside - node.position;
                    double dist2_to_outer     = vsize2_mm(direction_to_outer);
                    // Don't move towards the outside if the line to it is cut by the contour (the support may
                    // intersect the object), or if it is impossible to move to the build plate.
                    if (is_line_cut_by_contour(node.position, to_outside) || dist2_to_outer > max_move_distance2 * sqr(double(obj_layer_nr)) ||
                        ! is_inside_ex(avoidance_next, node.position)) {
                        // Try to move out of the lower layer instead.
                        Point          candidate_vertex         = node.position;
                        const coordf_t max_move_between_samples = max_move_distance + RADIUS_SAMPLE_RESOLUTION + EPSILON;
                        if (move_out_expolys(get_collision(next_radius, obj_layer_nr_next), candidate_vertex, max_move_between_samples, max_move_between_samples)) {
                            direction_to_outer = candidate_vertex - node.position;
                            dist2_to_outer     = vsize2_mm(direction_to_outer);
                        } else {
                            direction_to_outer = Point(0, 0);
                            dist2_to_outer     = 0;
                        }
                    }
                    Point movement;
                    if (node.is_sharp_tail && node.dist_mm_to_top < 3)
                        movement = normal(node.skin_direction, scale_(get_max_move_dist(&node)));
                    else if (dist2_to_outer > 0)
                        movement = normal(direction_to_outer, scale_(get_max_move_dist(&node)));
                    else
                        movement = normal(move_to_neighbor_center, scale_(get_max_move_dist(&node)));
                    next_layer_vertex += movement;

                    const ExPolygons &next_collision = get_collision(0, obj_layer_nr_next);
                    const bool        to_buildplate  = ! is_inside_ex(m_layer_outlines[obj_layer_nr_next], next_layer_vertex);
                    // Don't increase the radius if the next node would partially collide with the object.
                    to_outside         = projection_onto(next_collision, next_layer_vertex);
                    direction_to_outer = to_outside - node.position;
                    PendingNode pending;
                    pending.position                  = next_layer_vertex;
                    pending.distance_to_top           = node.distance_to_top + 1;
                    pending.support_roof_layers_below = node.support_roof_layers_below - (node.distance_to_top >= 0 ? 1 : 0);
                    pending.to_buildplate             = to_buildplate;
                    pending.parent                    = p_node;
                    pending.clamp_radius              = true;
                    pending.parent_radius             = node.radius;
                    pending.dist_to_outer             = unscale_mm(direction_to_outer.cast<double>().norm());
                    out.pending.emplace_back(std::move(pending));
                }
            });
            // Apply the recorded side effects in node order.
            for (size_t node_idx = 0; node_idx < nodes_vec.size(); ++ node_idx) {
                PassTwoResult &out = pass2_results[node_idx];
                for (PendingNode &pending : out.pending) {
                    SupportNode *next_node = create_node(pending.position, pending.distance_to_top, int(obj_layer_nr_next),
                        pending.support_roof_layers_below, pending.to_buildplate, pending.parent, print_z_next, height_next);
                    if (pending.zero_max_move)
                        next_node->max_move_dist = 0;
                    if (pending.has_overhang)
                        next_node->overhang = std::move(pending.overhang);
                    if (pending.clamp_radius) {
                        next_node->radius = std::max(pending.parent_radius, std::min(next_node->radius, pending.dist_to_outer));
                        get_max_move_dist(next_node);
                    }
                    m_contact_nodes[layer_nr_next].push_back(next_node);
                }
                if (out.unsupported_leaf)
                    unsupported_branch_leaves.push_front({ layer_nr, nodes_vec[node_idx].second });
                if (out.invalidate)
                    nodes_vec[node_idx].second->valid = false;
            }
        }

        // Prune all branches that couldn't find support on either the model or the build plate.
        for (; ! unsupported_branch_leaves.empty(); unsupported_branch_leaves.pop_back()) {
            const auto &entry  = unsupported_branch_leaves.back();
            SupportNode *i_node = entry.second;
            for (; i_node != nullptr; i_node = i_node->parent) {
                size_t i_layer = size_t(i_node->obj_layer_nr);
                if (i_node->parent) {
                    i_node->parent->child = i_node->child;
                    for (SupportNode *parent : i_node->parents)
                        if (parent->child == i_node)
                            parent->child = i_node->child;
                }
                if (i_node->child) {
                    i_node->child->parent = i_node->parent;
                    auto it = std::find(i_node->child->parents.begin(), i_node->child->parents.end(), i_node);
                    if (it != i_node->child->parents.end())
                        i_node->child->parents.erase(it);
                    append(i_node->child->parents, i_node->parents);
                }
                // Mark to be deleted.
                i_node->is_processed = true;
                for (SupportNode *neighbour : i_node->merged_neighbours)
                    if (neighbour && ! neighbour->is_processed)
                        unsupported_branch_leaves.push_front({ i_layer, neighbour });
            }
        }
        for (auto &nodes : m_contact_nodes)
            nodes.erase(std::remove_if(nodes.begin(), nodes.end(), [](const SupportNode *node) { return node->is_processed; }), nodes.end());
    }
}

void OrcaTreeSupport::smooth_nodes()
{
    for (auto &nodes : m_contact_nodes)
        for (SupportNode *node : nodes)
            node->is_processed = false;

    const float max_move = float(scale_(m_support_line_width / 2));
    // If the branch is very tall, the tip also needs an extra wall.
    const float thresh_tall_branch = 100;
    const float thresh_dist_to_top = 30;

    for (size_t layer_nr = 0; layer_nr < m_contact_nodes.size(); ++ layer_nr) {
        for (SupportNode *node : m_contact_nodes[layer_nr]) {
            if (node->is_processed)
                continue;
            std::vector<Point>        pts;
            std::vector<double>       radii;
            std::vector<SupportNode*> branch;
            SupportNode *p_node       = node;
            float        total_height = 0;
            // Add a fixed head if it's not a polygon node: polygon nodes may move a lot, making the nodes in between jump.
            if (node->child && node->child->type != NodeType::Polygon) {
                pts.push_back(node->child->position);
                radii.push_back(node->child->radius);
                branch.push_back(node->child);
                total_height += float(node->child->height);
            }
            do {
                pts.push_back(p_node->position);
                radii.push_back(p_node->radius);
                branch.push_back(p_node);
                total_height += float(p_node->height);
                p_node = p_node->parent;
            } while (p_node && ! p_node->is_processed);
            if (pts.size() < 3)
                continue;

            std::vector<Point>  pts1   = pts;
            std::vector<double> radii1 = radii;
            const int iterations = 100;
            for (int k = 0; k < iterations; ++ k) {
                for (size_t i = 1; i < pts.size() - 1; ++ i) {
                    Point pt  = (pts[i - 1] + pts[i] + pts[i + 1]) / 3;
                    pts1[i]   = pt;
                    radii1[i] = (radii[i - 1] + radii[i] + radii[i + 1]) / 3;
                    if (k == iterations - 1) {
                        branch[i]->position     = pt;
                        branch[i]->radius       = radii1[i];
                        branch[i]->movement     = (pts[i + 1] - pts[i - 1]) / 2;
                        branch[i]->is_processed = true;
                        if (branch[i]->parents.size() > 1 || branch[i]->movement.x() > max_move || branch[i]->movement.y() > max_move ||
                            (total_height > thresh_tall_branch && branch[i]->dist_mm_to_top < thresh_dist_to_top))
                            branch[i]->need_extra_wall = true;
                    }
                }
                if (k < iterations - 1) {
                    std::swap(pts, pts1);
                    std::swap(radii, radii1);
                } else {
                    for (size_t i = 1; i + 1 < branch.size(); ++ i)
                        if (branch[i - 1]->need_extra_wall && branch[i + 1]->need_extra_wall)
                            branch[i]->need_extra_wall = true;
                }
            }
        }
        m_throw_on_cancel();
    }
}

std::vector<LayerAreas> OrcaTreeSupport::draw_circles()
{
    std::vector<LayerAreas> out(m_layer_count);
    const coordf_t branch_radius        = m_base_radius;
    const coordf_t branch_radius_scaled = scale_(branch_radius);
    const bool     on_buildplate_only   = m_buildplate_only;
    // Square support if there are too many nodes per layer, as circles take much longer to compute.
    const bool     square_support       = m_avg_node_per_layer > 200;
    const int      circle_resolution    = square_support ? 4 : 100;
    Polygon        branch_circle;
    for (int i = 0; i < circle_resolution; ++ i) {
        double angle = double(i) / circle_resolution * TAU + (square_support ? M_PI_4 : 0.);
        branch_circle.points.emplace_back(coord_t(cos(angle) * branch_radius_scaled), coord_t(sin(angle) * branch_radius_scaled));
    }
    const size_t   top_interface_layers   = m_roof_layers;
    const int      top_base_interface_layers = m_top_base_interface_layers;
    const coordf_t line_width_scaled      = scale_(m_support_line_width);

    tbb::parallel_for(tbb::blocked_range<size_t>(0, m_layer_heights.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t layer_nr = range.begin(); layer_nr < range.end(); ++ layer_nr) {
            const std::vector<SupportNode*> &curr_layer_nodes = m_contact_nodes[layer_nr];
            if (curr_layer_nodes.empty())
                continue;
            const size_t obj_layer_nr = m_layer_heights[layer_nr].obj_layer_nr;
            ExPolygons base_areas, roof_areas, roof_base_areas, roof_1st_layer, roof_gap_areas;
            ExPolygons collision_sharp_tails, collision_base;
            auto get_collision_here = [&](bool sharp_tail) -> ExPolygons& {
                ExPolygons &collision = sharp_tail ? collision_sharp_tails : collision_base;
                if (collision.empty()) {
                    collision = offset_ex(m_layer_outlines[obj_layer_nr], float(scale_(sharp_tail ? m_top_z_distance : m_xy_distance)));
                    // The top layers may be too close to the interface with a very small overhang angle.
                    if (m_top_z_distance > EPSILON) {
                        double accum_height = 0;
                        for (size_t layer_id = obj_layer_nr + 1; layer_id < m_layer_outlines.size(); ++ layer_id) {
                            accum_height += m_object.get_layer(int(layer_id))->height;
                            if (accum_height > m_top_z_distance)
                                break;
                            collision = union_ex(collision, offset_ex(m_layer_outlines[layer_id], float(scale_(m_top_z_distance))));
                        }
                    }
                }
                return collision;
            };

            // Draw the support areas and add the roofs to the support roof instead of the normal areas.
            for (const SupportNode *p_node : curr_layer_nodes) {
                const SupportNode &node = *p_node;
                ExPolygons area;
                if (node.type == NodeType::Polygon || (node.distance_to_top < 0 && ! node.is_sharp_tail)) {
                    // Generate directly from the overhang polygon for a normal (hybrid) node or a virtual node.
                    if (node.overhang.contour.points.size() > 100 || node.overhang.holes.size() > 1)
                        area.emplace_back(node.overhang);
                    else
                        area = offset_ex(node.overhang, float(scale_(m_xy_distance)));
                    area = diff_ex(area, get_collision_here(node.is_sharp_tail && node.distance_to_top <= 0));
                } else {
                    Polygon circle(branch_circle);
                    double  scale = node.radius / branch_radius;
                    double  moveX = node.movement.x() / (scale * branch_radius_scaled);
                    double  moveY = node.movement.y() / (scale * branch_radius_scaled);
                    if (! square_support && std::abs(moveX) > 0.001 && std::abs(moveY) > 0.001) {
                        // Draw an ellipse along the movement direction.
                        const double vsize_inv = 0.5 / (0.01 + std::sqrt(moveX * moveX + moveY * moveY));
                        const double matrix[4] = {
                            scale * (1 + moveX * moveX * vsize_inv), scale * (0 + moveX * moveY * vsize_inv),
                            scale * (0 + moveX * moveY * vsize_inv), scale * (1 + moveY * moveY * vsize_inv),
                        };
                        for (size_t i = 0; i < branch_circle.points.size(); ++ i) {
                            const Point &v = branch_circle.points[i];
                            circle.points[i] = node.position + Point(coord_t(matrix[0] * v.x() + matrix[1] * v.y()), coord_t(matrix[2] * v.x() + matrix[3] * v.y()));
                        }
                    } else {
                        for (size_t i = 0; i < circle.points.size(); ++ i)
                            circle.points[i] = scaled_point(branch_circle.points[i], scale) + node.position;
                    }
                    if (obj_layer_nr == 0) {
                        // Automatic brim for the trunks: taller trees get wider feet.
                        double brim_width = std::max(MIN_BRANCH_RADIUS_FIRST_LAYER,
                            std::min(node.radius + node.dist_mm_to_top / (scale * branch_radius) * 0.5, MAX_BRANCH_RADIUS_FIRST_LAYER) - node.radius);
                        Polygons tmp = offset(circle, float(scale_(brim_width)));
                        if (! tmp.empty())
                            circle = tmp.front();
                    }
                    area = avoid_object_remove_extra_small_parts(ExPolygon(circle), get_collision_here(node.is_sharp_tail && node.distance_to_top <= 0));
                    // Merge the overhang to get a smoother interface surface. Not with buildplate only,
                    // as some nodes underneath may have been deleted.
                    if (top_interface_layers > 0 && node.support_roof_layers_below > 0 && ! on_buildplate_only && ! node.is_sharp_tail) {
                        if (node.overhang.contour.points.size() > 100 || node.overhang.holes.size() > 1)
                            area.emplace_back(node.overhang);
                        else
                            append(area, offset_ex(node.overhang, float(scale_(m_xy_distance))));
                    }
                }

                if (obj_layer_nr > 0 && node.distance_to_top < 0)
                    append(roof_gap_areas, std::move(area));
                else if (obj_layer_nr > 0 && node.support_roof_layers_below == 1 && ! node.is_sharp_tail)
                    append(roof_1st_layer, std::move(area));
                else if (obj_layer_nr > 0 && node.support_roof_layers_below > 1 && ! node.is_sharp_tail)
                    append(node.support_roof_layers_below <= top_base_interface_layers ? roof_base_areas : roof_areas, std::move(area));
                else
                    append(base_areas, std::move(area));
            }

            // Join the roof segments.
            roof_areas      = diff_ex(closing_ex(roof_areas, float(line_width_scaled)), get_collision_here(false));
            roof_base_areas = diff_ex(closing_ex(roof_base_areas, float(line_width_scaled)), get_collision_here(false));
            if (! roof_base_areas.empty() && ! roof_areas.empty())
                roof_base_areas = diff_ex(roof_base_areas, roof_areas);
            roof_1st_layer  = diff_ex(closing_ex(roof_1st_layer, float(line_width_scaled)), get_collision_here(false));
            // The first roof layer and the roof areas may intersect.
            roof_1st_layer  = diff_ex(roof_1st_layer, roof_areas);
            // The lowest roof layer is printed as a base interface layer when there are any.
            if (top_base_interface_layers > 0)
                roof_base_areas = union_ex(diff_ex(roof_base_areas, roof_1st_layer), roof_1st_layer);
            else
                append(roof_areas, std::move(roof_1st_layer));
            ExPolygons roofs = roof_areas;
            append(roofs, roof_base_areas);
            append(roofs, roof_gap_areas);
            base_areas = diff_ex(union_ex(base_areas), roofs);
            if (square_support)
                base_areas = simplify_expolygons(base_areas, line_width_scaled / 2);

            // Bottom Z gap over the part below, then bottom interface layers over it.
            if (! base_areas.empty() && ! on_buildplate_only && obj_layer_nr > 0) {
                const Layer   &layer    = *m_object.get_layer(int(obj_layer_nr));
                const coordf_t bottom_z = layer.bottom_z();
                Polygons trimming;
                for (int j = int(obj_layer_nr) - 1; j >= 0; -- j) {
                    const Layer &below = *m_object.get_layer(j);
                    if (below.print_z < bottom_z - m_bottom_gap + EPSILON)
                        break;
                    append(trimming, ExPoly::to_polygons(m_layer_outlines[j]));
                }
                if (! trimming.empty())
                    base_areas = diff_ex(base_areas, trimming);
                if (m_bottom_interface_layers > 0 && ! base_areas.empty()) {
                    ExPolygons floor;
                    for (size_t k = 0; k < m_bottom_interface_layers; ++ k) {
                        const coordf_t z = bottom_z - m_bottom_gap - k * m_layer_height;
                        // The highest object layer at or below z.
                        int j = int(obj_layer_nr) - 1;
                        while (j >= 0 && m_object.get_layer(j)->print_z > z + EPSILON)
                            -- j;
                        if (j < 0)
                            break;
                        append(floor, intersection_ex(base_areas, m_layer_outlines[j]));
                    }
                    if (! floor.empty()) {
                        floor      = union_ex(floor);
                        base_areas = diff_ex(base_areas, offset_ex(floor, 10.f));
                        out[obj_layer_nr].floor = std::move(floor);
                    }
                }
            }

            // Remove small holes.
            auto remove_small_holes = [](ExPolygons &expolys) {
                for (ExPolygon &expoly : expolys)
                    expoly.holes.erase(std::remove_if(expoly.holes.begin(), expoly.holes.end(), [](const Polygon &hole) {
                        Point size = bbox_size(Algorithms::Polygon::get_extents(hole));
                        return size.x() < scale_(2) && size.y() < scale_(2);
                    }), expoly.holes.end());
            };
            remove_small_holes(base_areas);
            remove_small_holes(roof_areas);
            remove_small_holes(roof_base_areas);

            out[obj_layer_nr].base      = std::move(base_areas);
            out[obj_layer_nr].roof      = std::move(roof_areas);
            out[obj_layer_nr].roof_base = std::move(roof_base_areas);
            m_throw_on_cancel();
        }
    });
    return out;
}

std::vector<LayerAreas> OrcaTreeSupport::generate()
{
    detect_overhangs();
    if (m_highest_overhang_layer == 0 && std::all_of(m_overhangs.begin(), m_overhangs.end(), [](const ExPolygons &o) { return o.empty(); }))
        return {};
    generate_contact_points();
    plan_layer_heights();
    drop_nodes();
    smooth_nodes();
    return draw_circles();
}

} // namespace

void orca_tree_support_generate(PrintObject &print_object, std::function<void()> throw_on_cancel)
{
    const Domain::SupportMaterialStyle style = print_object.config().get<Domain::SupportMaterialStyle>("support_material_style");
    std::vector<LayerAreas> areas;
    if (print_object.has_support()) {
        OrcaTreeSupport generator(print_object, style == Domain::SupportMaterialStyle::smsTreeSlim, style == Domain::SupportMaterialStyle::smsTreeHybrid, throw_on_cancel);
        areas = generator.generate();
    }

    const SlicingParameters &slicing_params = print_object.slicing_parameters();
    SupportParameters        support_params(print_object);
    support_params.with_sheath = true;

    SupportGeneratorLayerStorage layer_storage;
    SupportGeneratorLayersPtr    top_contacts;
    SupportGeneratorLayersPtr    bottom_contacts;
    SupportGeneratorLayersPtr    interface_layers;
    SupportGeneratorLayersPtr    base_interface_layers;
    SupportGeneratorLayersPtr    intermediate_layers;
    auto new_layer = [&layer_storage](SupporLayerType type, const Layer &layer, Polygons &&polygons) {
        SupportGeneratorLayer &out = layer_storage.allocate_unguarded(type);
        out.print_z  = layer.print_z;
        out.bottom_z = layer.bottom_z();
        out.height   = layer.height;
        out.polygons = std::move(polygons);
        return &out;
    };
    for (size_t layer_nr = 0; layer_nr < areas.size(); ++ layer_nr) {
        const Layer &layer = *print_object.get_layer(int(layer_nr));
        LayerAreas  &la    = areas[layer_nr];
        if (! la.base.empty())
            intermediate_layers.push_back(new_layer(SupporLayerType::Base, layer, ExPoly::to_polygons(union_ex(la.base))));
        ExPolygons interface = la.roof;
        append(interface, la.floor);
        if (! interface.empty())
            interface_layers.push_back(new_layer(SupporLayerType::TopInterface, layer, ExPoly::to_polygons(union_ex(interface))));
        if (! la.roof_base.empty())
            base_interface_layers.push_back(new_layer(SupporLayerType::Base, layer, ExPoly::to_polygons(union_ex(la.roof_base))));
    }
    if (intermediate_layers.empty() && interface_layers.empty() && base_interface_layers.empty() && slicing_params.raft_layers() == 0)
        return;

    SupportGeneratorLayersPtr raft_layers = generate_raft_base(print_object, support_params, slicing_params,
        top_contacts, interface_layers, base_interface_layers, intermediate_layers, layer_storage);
    generate_support_layers(print_object, raft_layers, bottom_contacts, top_contacts, intermediate_layers, interface_layers, base_interface_layers);
    generate_support_toolpaths(print_object.support_layers(), print_object.config(), support_params, slicing_params,
        raft_layers, bottom_contacts, top_contacts, intermediate_layers, interface_layers, base_interface_layers);
}

} // namespace FFFTreeSupport

} // namespace Slic3r

#ifndef slic3r_OrcaTreeSupport_hpp
#define slic3r_OrcaTreeSupport_hpp

#include <functional>

namespace Slic3r
{

class PrintObject;

namespace FFFTreeSupport
{

// Tree supports ported from OrcaSlicer (originally BambuStudio), the "Tree Slim" and
// "Tree Hybrid" styles. Contact points are sampled on the overhangs, dropped layer by layer
// towards the bed while merging along a minimum spanning tree, smoothed, then drawn as circles.
// Tree Hybrid supports large overhangs over free space with straight columns under the
// overhang instead of branches. The areas are printed like Organic supports.
void orca_tree_support_generate(PrintObject &print_object, std::function<void()> throw_on_cancel);

} // namespace FFFTreeSupport

} // namespace Slic3r

#endif // slic3r_OrcaTreeSupport_hpp

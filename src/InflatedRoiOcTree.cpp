#include "octomap_vpp/InflatedRoiOcTree.h"

namespace octomap_vpp
{

InflatedRoiOcTree::InflatedRoiOcTree(double resolution, double influence_radius, double max_roi_val) : octomap::OcTreeBase <InflatedRoiOcTreeNode>(resolution),
  influence_radius(influence_radius), max_roi_val(max_roi_val)
{
  ocTreeMemberInit.ensureLinking();
}

InflatedRoiOcTree::StaticMemberInitializer InflatedRoiOcTree::ocTreeMemberInit;

}

#include "octomap_vpp/InflatedRoiOcTree.h"

namespace octomap_vpp
{

InflatedRoiOcTree::InflatedRoiOcTree(double resolution) : octomap::OcTreeBase <InflatedRoiOcTreeNode>(resolution), max_roi_val(1.0f), influence_radius(0.5f)
{
  ocTreeMemberInit.ensureLinking();
}

InflatedRoiOcTree::StaticMemberInitializer InflatedRoiOcTree::ocTreeMemberInit;

}
